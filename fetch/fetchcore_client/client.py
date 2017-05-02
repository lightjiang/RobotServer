# Copyright 2014-2016 Fetch Robotics Inc.
# Authors: Michael Ferguson, Ian Danforth, Aaron Blasdel, Michael Hwang

## @file client.py Fetchcore client library

# Standard Library
import copy
import datetime
import json
import logging
import os
import requests
import socket
import yaml
from Queue import Queue

# Third Party

# Fetch
from fetchcore_client.settings import CORE_SERVER_URL
API_VERSION = "v1"

## Abstraction of requests. Main utility is for keeping track of a Session
#  object so we can keep authentication.
class RequestWrapper(object):

    def __init__(self, verify_ssl=True):
        self.session = requests.Session()
        self.session.verify = verify_ssl
        self.session.headers = {"User-Agent": "fetchcore.client.python",
                                "Accept": "application/json"}

    def get(self, *args, **kwargs):

        return self.session.get(*args, **kwargs)

    def post(self, *args, **kwargs):

        return self.session.post(*args, **kwargs)

    def put(self, *args, **kwargs):

        return self.session.put(*args, **kwargs)

    def delete(self, *args, **kwargs):

        return self.session.delete(*args, **kwargs)

# TODO: Add support for reconnecting and data batching during bad connectivity
## Client interface for communicating with fetchcore server.
#  All document creation
class FetchcoreClient(object):

    ## @param agentname The name of the agent to connect as.
    ## @param password The password of the agent to connect with.
    ## @param url Target URL of the server
    ## @param rw Wrapper for HTTP requests
    def __init__(self, agentname, password,
                 url=None, rw=None, verify_ssl=True,
                 on_connect=None):
        self.baseurl = url if url else CORE_SERVER_URL

        # Make sure baseurl ends with / for easy appending
        if self.baseurl[-1] != '/':
            self.baseurl += '/'

        self.apiurl = self.baseurl + "api/" + API_VERSION + "/"

        if not agentname:
            raise ValueError("Client initialized without a agentname.")
        if not password:
            raise ValueError("Client initialized without a password.")

        # We'll need to authenticate the client
        self.agentname = agentname
        self.password = password

        # Timeout parameters
        self.login_timeout = 5.0
        self.request_timeout = 30.0

        # Callback for reconnection
        self.on_connect = on_connect

        # Authentication tracking
        self._is_connected = False
        self._handle_auth = False

        # Request tracking
        self.connect_iter = 0
        self.connect_iter_loop = 20

        # Batch for requests that were attempted but were not sent due to
        # connectivity errors
        self.request_cache = Queue()

        # Allow dependencies to be injected
        self.rw = rw if rw else RequestWrapper(verify_ssl=verify_ssl)
        self.request_array = {'POST': self.rw.post,
                              'PUT': self.rw.put,
                              'GET': self.rw.get,
                              'DELETE': self.rw.delete}

    ## @brief Get the password for this client
    ## This exists so that we can override the password in the reactor
    def get_password(self):
        return self.password

    ## Get the login timeout for this client.
    #  This exists so we can override the login timeout with a rosparam
    #  in the reactor.
    def get_login_timeout(self):
        return self.login_timeout

    ## Get the request timeout for this client.
    #  This exists so we can override the request timeout with a rosparam
    #  in the reactor.
    def get_request_timeout(self):
        return self.request_timeout

    ## Log into the fetchcore service. Cookies (including authentication)
    #  will be stored in the requests session.
    ## @param handle_auth Whether to handle authentication failure with
    #  authorization checks
    def connect(self, handle_auth=False):
        if self.connect_iter != 0:
            self.connect_iter = ((self.connect_iter + 1)
                                 % self.connect_iter_loop)
            return None
        else:
            self.connect_iter = ((self.connect_iter + 1)
                                 % self.connect_iter_loop)
        # Calling connect directly will set behavior for future reconnects
        self._handle_auth = handle_auth
        url = self.baseurl + "login"
        authbody = {"agentname": self.agentname, "password": self.get_password()}
        try:
            response = self.rw.post(
                url, data=authbody, timeout=self.get_login_timeout())
        except (requests.exceptions.ConnectionError, requests.exceptions.Timeout, socket.timeout) as err:
            # This means that we couldn't establish a connection to the server
            self._is_connected = False
            # This is going to spam a bunch in normal reactor operation, so
            # it's a debug.
            print datetime.datetime.now(), "Attempted to connect to server, and received error:", err
            return None

        # TODO: Consider rearranging logic for performance increase
        # Status code handling
        if handle_auth:
            if response.status_code < 400:
                self._is_connected = True
                self.connect_iter = 0
                if self.on_connect:
                    self.on_connect()
            elif response.status_code == 403:
                # The agent's record exists, but its authorization has been
                # rejected.
                self._is_connected = False
            elif response.status_code == 401:
                # This means that this agent's record doesn't exist, and we
                # want it to, so create a record with pending authorization.
                self.create_own_unauth_agent_doc()
            else:
                # Catch-all for other statuses
                response.raise_for_status()
        else:
            if response.status_code >= 400:
                if response.status_code == 401:
                    # Set flag in case we catch the exception and continue
                    self._is_connected = False
                response.raise_for_status()
            else:
                self._is_connected = True
                self.connect_iter = 0
        return response

    ## Log out from the fetchcore system (de-authenticate)
    def disconnect(self):
        if self._is_connected:
            url = self.baseurl + "logout"
            # Logging out is just a GET request to the logout handler
            self.rw.get(url)
            self._is_connected = False

    ## Attempt to login to the fetchcore service and process requests cached
    #  during connection failure.
    def reconnect(self):
        connect_resp = self.connect(handle_auth=self._handle_auth)
        if self._is_connected and not self.request_cache.empty():
            # TODO: Add log about sending batched updates if blocking?
            # TODO: Consider using blocked queue in separate thread?
            while not self.request_cache.empty():
                req = self.request_cache.get()
                self.make_request(req['request_type'],
                                  req['url'],
                                  **req['kwargs'])
        return connect_resp

    ## @name Document prerequest processing.
    #  These methods are for converting JSON documents to JSON strings for
    #  requests to the HTTP server.
    ## @{

    ## Wrapper method for requests that has preprocessing for connection
    #  behavior and request caching.
    ## @param request_type The type of request (GET/POST/PUT/DELETE)
    ## @param url The URL to send against
    ## @param store_req_on_failure Whether to cache this request or not.
    #  FUTURE TODO: Implement and test this "caching" properly
    ## @param try_reconnect Whether to attempt reconnect behavior if we
    #  think we aren't connected.
    ## @param kwargs Additional kwargs for the request method
    def make_request(self, request_type, url,
                     store_req_on_failure=False,
                     try_reconnect=True,
                     **kwargs):
        if not self._is_connected and try_reconnect:
            resp = self.reconnect()
            if not resp:
                # Tried to connect and didn't get anything
                return None
            elif resp.status_code >= 400:
                # We're unauthorized, so don't go through with request.
                return None
        # uppercase request type
        request_type = request_type.upper()
        try:
            response = self.request_array[request_type](
                url, timeout=self.get_request_timeout(), **kwargs)
        except (requests.exceptions.ConnectionError, requests.exceptions.Timeout, socket.timeout):
            # Can't connect to the server for whatever reason
            self._is_connected = False
            if store_req_on_failure:
                self.request_cache.put({'request_type': request_type,
                                        'url': url,
                                        'kwargs': kwargs})
            return None
        if response.status_code == 401 and self._handle_auth:
            # If we get an unauthenticated response and we are handling auth,
            # then mark us for a reconnect and don't raise an error
            self._is_connected = False
        elif response.status_code >= 400:
            response.raise_for_status()
        return response

    ## POST a document using the wrapper specified in self.rw
    ## @param url The URL to access.
    ## @param json_doc A dictionary that will be converted to a JSON string
    #  for the data field in the request.
    ## @param kwargs Additional key-value pairs that the request should use
    def post_document(self, url, json_doc, **kwargs):
        try:
            data = json.dumps(json_doc)
        except ValueError:
            # Un-JSONable, so make it a string
            data = str(json_doc)
        response = self.make_request('post', url, data=data, **kwargs)
        return response

    ## PUT a document using the wrapper specified in self.rw
    ## @param url The URL to access.
    ## @param json_doc A dictionary that will be converted to a JSON string
    #  for the data field in the request.
    ## @param kwargs Additional key-value pairs that the request should use
    def put_document(self, url, json_doc, **kwargs):
        try:
            data = json.dumps(json_doc)
        except ValueError:
            data = json_doc
        response = self.make_request('put', url, data=data, **kwargs)
        return response

    ## GET a document using the wrapper specified in self.rw
    ## @param url The URL to access.
    ## @param params Key-value pairs corresponding to the parameter arguments
    #  given in the HTTP request.
    def get_document(self, url, try_reconnect=True, **params):
        response = self.make_request('get', url,
                                     try_reconnect=try_reconnect,
                                     params=params)
        if not response:
            return None
        try:
            doc = json.loads(response.text)
        except ValueError:
            doc = response.text
        return doc

    ## DELETE a document using the wrapper specified in self.rw
    ## @param url The URL to access.
    ## @param kwargs Additional key-value pairs that the request should use
    def delete_document(self, url, **kwargs):
        response = self.make_request('delete', url, **kwargs)
        return response

    ## @}

    ## @name task_api
    #  These are clientside methods for dealing with tasks through the
    #  HTTP interface. All document validation is done server-side, so most
    #  documents can be defined by key-value pairs as arguments.
    #  A good way to upload tasks are to create an appropriate JSON file,
    #  then pass it in to the proper method using an unwrap (**) prefix.
    #  Example: create_task(**task_json)
    ## @{

    ## @brief Create a task document and upload it to the server.
    def create_task(self, **task):
        url = self.apiurl + "tasks/"
        return self.post_document(url, task)

    ## @brief Update a task document. Other than the ID, only the fields
    #  being updated need to be provided. The document must already exist on
    #  the database or the request will fail.
    def update_task(self, **task):
        try:
            url = self.apiurl + "tasks/" + task['task_id']
        except KeyError:
            raise ValueError("Cannot update a task without a task ID.")
        return self.put_document(url, task)

    def get_task(self, task_id):
        url = self.apiurl + "tasks/" + task_id
        return self.get_document(url)

    ## @brief Gets a list of the most recent tasks.
    def get_tasks(self, **kwargs):
        url = self.apiurl + "tasks/"
        return self.get_document(url, **kwargs)['results']

    def delete_task(self, task_id):
        # Tasks have to be non-working to delete them
        task = self.get_task(task_id)
        if task.get("task_status") == "WORKING":
            task["task_status"] = "CANCELED"
            self.update_task(**task)

        url = self.apiurl + "tasks/" + task_id
        return self.delete_document(url)

    ## @brief Get the next queued task for agentname.
    ## @returns A single QUEUED task document, if it exists.
    ## @returns None otherwise.
    def get_queued_task(self, agentname=None):
        if not agentname:
            agentname = self.agentname
        url = self.apiurl + "tasks/"
        recv_docs = self.get_document(url, agent=agentname, status="QUEUED", full=True)
        rows = recv_docs["results"] if recv_docs else []
        # TODO: It seems couchdb is returning these in an inconsistent order
        #       we should definitely dig in more later, but for now:
        queued = None
        for row in rows:
            if queued:
                lut = row["last_updated_timestamp"]
                queued_lut = queued["last_updated_timestamp"]
                if lut < queued_lut:
                    queued = row
            else:
                queued = row
        return queued

    ## @}

    ## @brief Get a list of valid template types fetchcore currently supports.
    def get_templates(self):
        url = self.apiurl + "templates"
        return self.get_document(url)

    ## @name task_templates_api
    #  These are clientside methods for dealing with task templates through the
    #  HTTP interface. All document validation is done server-side, so most
    #  documents can be defined by key-value pairs as arguments.
    #  A good way to upload templates are to create an appropriate JSON file,
    #  then pass it in to the proper method using an unwrap (**) prefix.
    #  Example: create_task_template(**task_template_json)
    ## @{

    ## @brief Create a task template document and upload it to the server.
    def create_task_template(self, **task_template):
        url = self.apiurl + "templates/tasks/"
        return self.post_document(url, task_template)

    ## @brief Update a task template document. Other than the ID, only the
    #  fields being updated need to be provided. The document must already exist
    #  on the database or the request will fail.
    def update_task_template(self, **task_template):
        try:
            url = self.apiurl + \
                  "templates/tasks/" + task_template['task_template_id']
        except KeyError:
            raise ValueError(
                "Cannot update a task template without a task template ID.")
        return self.put_document(url, task_template)

    def get_task_template(self, task_template_id):
        url = self.apiurl + "templates/tasks/" + task_template_id
        return self.get_document(url)

    def get_task_templates(self, **kwargs):
        url = self.apiurl + "templates/tasks/"
        return self.get_document(url, **kwargs)['results']

    def delete_task_template(self, task_template_id):
        url = self.apiurl + "templates/tasks/" + task_template_id
        return self.delete_document(url)

        ## @}

    ## @name agent_api
    #  These are clientside methods for dealing with agents through the HTTP
    #  interface. It mirrors the Task API documentation (see above).
    ## @{

    ## @brief Create an agent and upload it to the server.
    def create_agent(self, **agent):
        url = self.apiurl + "agents/"
        return self.post_document(url, agent)

    ## @brief Update an agent document. Other than the ID, only the fields
    #  being updated need to be provided. The document must already exist on
    #  the database or the request will fail.
    def update_agent(self, **agent):
        try:
            url = self.apiurl + "agents/" + agent["agentname"]
        except KeyError:
            raise ValueError("Cannot update an agent without an agent ID.")
        return self.put_document(url, agent)

    def get_agent(self, agent_id):
        url = self.apiurl + "agents/" + agent_id
        return self.get_document(url)

    def get_agents(self, **params):
        url = self.apiurl + "agents/"
        return self.get_document(url, **params)['rows']

    def delete_agent(self, agent_id):
        url = self.apiurl + "agents/" + agent_id
        return self.delete_document(url)

    ## @brief Get agent document corresponding to client's agentname
    def get_own_agent_doc(self):
        return self.get_agent(self.agentname)

    def update_own_agent_doc(self, **agent):
        if 'agentname' not in agent:
            agent['agentname'] = self.agentname
        try:
            return self.update_agent(**agent)
        except requests.exceptions.HTTPError, e:
            # We shouldn't be hitting unexpected errors for "own" methods
            if str(e).startswith('401'):
                # This can only mean that our auth status was set to PENDING
                # from AUTHORIZED
                self._is_connected = False
            elif str(e).startswith('403'):
                # This can only mean that our auth status was set to DENIED
                # from AUTHORIZED (?)
                self._is_connected = False
            elif str(e).startswith('404'):
                # This means that our agent record was deleted somehow and we
                # should prompt a relog
                self._is_connected = False
            else:
                # Any other error that was going to be raised, we should raise
                raise e

    def update_own_location(self, **location):
        url = self.apiurl + "agents/" + self.agentname
        location_doc = {'agent_state': location}
        return self.put_document(url, location_doc)

    def get_agent_tasks(self, agent_id, status=None):
        url = self.apiurl + "tasks/"
        return self.get_document(url, agent=agent_id, status=status)['results']


    ## @}

    ## @name auth_api
    #  These are clientside methods for dealing with unauthorized agent
    #  records. Only "robot" level agents can be unauthorized.
    #  Although authorization status is part of the agent document, it can
    #  only be handled through the auth endpoint.
    ## @{

    ## Get an agent's doc only with information pertaining to authorization
    #  status.
    def get_agent_auth_status(self, agentname):
        url = self.apiurl + "auth/" + agentname
        return self.get_document(url)

    ## Set **agentname**'s authorization status to **auth_state**.
    def set_agent_auth_status(self, agentname, auth_state):
        url = self.apiurl + "auth/" + agentname
        auth_info = {'auth_info': {'auth_state': auth_state}}
        return self.put_document(url, auth_info)

    ## Create an agent doc with a pending authorization status
    def create_unauth_agent_doc(self, agentname, pw, **kwargs):
        url = self.apiurl + "auth/"
        agent = {'agentname': agentname, 'pw': pw}
        agent.update(kwargs)
        try:
            return self.post_document(url, agent, try_reconnect=False)
        except requests.exceptions.HTTPError as e:
            # We already tried to create one, so we continue
            if str(e).startswith('409'):
                return True
            else:
                raise e

    ## Get this agent's authorization information.
    def get_own_auth_status(self):
        url = self.apiurl + "auth/" + self.agentname
        return self.get_document(url, try_reconnect=False)

    ## Create a pending authorization document for this agent.
    def create_own_unauth_agent_doc(self, **kwargs):
        try:
            return self.create_unauth_agent_doc(self.agentname,
                                                self.get_password(),
                                                **kwargs)
        except requests.exceptions.HTTPError as e:
            # We already tried to create one, so we continue
            if str(e).startswith('409'):
                return True
            else:
                raise e
    ## @}

    ## @name log_api
    #  Clientside methods related to logging.
    ## @{

    ## @brief Create a log and upload it to the server.
    def create_log(self, **log):
        url = self.apiurl + "logs/"
        return self.post_document(url, log)

    ## @}

    ## @name action_api
    #  Clientside methods for dealing with action definitions through the HTTP
    #  interface.
    ## @{

    def create_action_definition(self, **actdef):
        url = self.apiurl + "actions/"
        return self.post_document(url, actdef)

    def update_action_definition(self, **actdef):
        try:
            url = self.apiurl + "actions/" + actdef['action_name']
        except KeyError:
            raise ValueError("Cannot update an action without a provided "
                             "action_name field.")
        return self.put_document(url, actdef)

    def get_action_definition(self, action_name):
        url = self.apiurl + "actions/" + action_name
        return self.get_document(url)

    def get_available_action_definitions(self):
        url = self.apiurl + "actions/"
        doc = self.get_document(url)
        result = {}
        for action in doc["results"]:
            result[action["action_name"]] = action
        return result

    def delete_action_definition(self, action_name):
        url = self.apiurl + "actions/" + action_name
        return self.delete_document(url)

    ## @}

    ## @name map_api
    #  Clientside methods for dealing with maps through the HTTP interface.
    ## @{

    MAP_TYPES = {'.yaml': {'protocol': 'text/yaml', 'mode': 'r'},
                 '.png': {'protocol': 'image/png', 'mode': 'rb'}}

    EXPECTED_MAP_FILES = {'map_image': 'map.png',
                          'keepout_image': 'keepout.png',
                          'speedlimit_image': 'speedlimit.png'}

    TEMPLATE_YAML = {'occupied_thresh': 0.65,
                     'free_thresh': 0.196,
                     'negate': 0,
                     'origin': [0.0, 0.0, 0.0]}

    ## @returns The id of the default map on the server
    ## @raises ValueError if default map name is equivalent to None
    def get_default_map_id(self, include_timestamp=False):
        if not include_timestamp:
            default_map_id = self.get_setting_for_component(
                'map', 'default_map')
        else:
            default_map_id, setting_timestamp = self.get_setting_for_component(
                'map', 'default_map', True)
        if not default_map_id:
            # For when we can't find a name in the default map.
            # TODO: Consider moving this elsewhere in logic?
            raise ValueError("Default map name was not retrieved.")
        if not include_timestamp:
            return default_map_id
        else:
            return default_map_id, setting_timestamp

    ## Set the default map ID on the server.
    def set_default_map_id(self, map_id):
        return self.update_setting_for_component('map', 'default_map', map_id)

    ## Get the default map from the server and return it as a normal map GET
    ## @returns The name of the default map.
    def get_default_map(self):
        default_map_id = self.get_default_map_id()
        return self.get_map(default_map_id)

    ## Get the default map from the server and save it to **map_folder**.
    ## @returns The map_id of the default map
    def save_default_map_to_folder(self, map_folder):
        default_map_id = self.get_default_map_id()
        mapinfo = self.save_map_to_folder(default_map_id, map_folder)
        return (default_map_id, mapinfo)

    ## @returns A list of the names of all available maps on the server
    def get_available_map_names(self):
        url = self.apiurl + "maps/"
        doc = self.get_document(url)
        try:
            return [x["mapname"] for x in doc['results']]
        except (KeyError, TypeError):
            return []

    ## @returns A list of the IDs of all available maps on the server
    def get_available_map_ids(self):
        url = self.apiurl + "maps/"
        doc = self.get_document(url)
        try:
            return [x["map_id"] for x in doc['results']]
        except (KeyError, TypeError):
            return []

    ## Download the map identified by **map_id** and its components.
    ## @param map_id An identifier for the map
    ## @returns A tuple consisting of the map info and a dictionary containing
    #           the component data, respectively
    def get_map(self, map_id):
        if not map_id:
            raise ValueError("Invalid value '%s' provided for argument "
                             "'map_id'." % map_id)
        url = self.apiurl + 'maps/' + str(map_id)
        attach_dict = {}
        for attach_name in self.EXPECTED_MAP_FILES:
            try:
                attachment = self.get_map_attachment(map_id, attach_name)
                attach_dict[attach_name] = attachment
            except requests.exceptions.HTTPError:
                continue
        # grab the general map info
        mapinfo = self.get_document(url)
        return mapinfo, attach_dict

    ## Download a map and save its components under base_folder/map_id
    ## @param map_id An identifier for the map, which can be either its
    #                 document ID (guaranteed unique) or its given name.
    def save_map_to_folder(self, map_id, base_folder):
        map_id = str(map_id)

        mapinfo, attachments = self.get_map(map_id)

        if not mapinfo or not attachments:
            logging.error("FetchcoreClient: Could not find info or attachments"
                          " map %s", map_id)
            return mapinfo

        if not base_folder.endswith("/"):
            base_folder += "/"
        savepath = base_folder + map_id + "/"

        if not os.path.exists(base_folder):
            os.makedirs(base_folder)

        if not os.path.exists(savepath):
            os.makedirs(savepath)

        # Save the attachments
        for key in attachments:
            basename = str(key).split("_")[0]
            try:
                # We should only be saving PNGs from attachments
                with open(savepath + basename + '.png', 'wb') as f:
                    f.write(attachments[key])
                # Generate a corresponding yaml file
                new_yaml = copy.deepcopy(self.TEMPLATE_YAML)
                new_yaml['image'] = basename + '.png'
                # Grab resolution field from mapinfo, or set default value
                if 'metadata' in mapinfo and \
                        'resolution' in mapinfo["metadata"]:
                    new_yaml['resolution'] = \
                        mapinfo['metadata']['resolution']
                else:
                    new_yaml['resolution'] = 0.05
                if basename == 'speedlimit':
                    new_yaml['mode'] = 'raw'
                # Save the yaml
                with open(savepath + basename + '.yaml', 'w') as f:
                    yaml.dump(new_yaml, f)
            except TypeError:
                logging.error("FetchcoreClient: Failed saving map %s to %s",
                              map_id, savepath)

        return mapinfo

    ## Retrieve data for an attachment to **map_id** specified by
    #  **attach_name**.
    ## @param map_id An identifier for the map
    ## @returns The raw data for the attachment (if retrieved).
    def get_map_attachment(self, map_id, attach_name):
        url = self.apiurl + "maps/%s/%s" % (map_id, attach_name)
        response = self.make_request("get", url)

        return response.content

    ## Upload a map as **map_id** from **map_folder**. It will be set as the
    #  default map if **set_default** is True.
    ## @param map_id The display name for the map
    ## @param map_id The document ID to upload the map under
    def upload_map(self, mapname, map_folder, set_default=False,
                   in_progress=False, map_id=None, buildmap_task_id=None,
                   initialized=False):
        url = self.apiurl + 'maps/'
        files, metadata = self._get_mapfiles_from_folder(map_folder)
        mapinfo = {'mapname': mapname,
                   'in_progress': in_progress,
                   'metadata': metadata,
                   'initialized': initialized,
                   'created_by': "AGENT-" + self.agentname}
        if map_id:
            mapinfo["map_id"] = map_id
        if buildmap_task_id:
            mapinfo["buildmap_task_id"] = buildmap_task_id
        if files:
            post_data = {'data': json.dumps(mapinfo)}
        else:
            post_data = json.dumps(mapinfo)
        response = self.make_request("post", url, files=files, data=post_data)
        if set_default:
            json_dict = json.loads(response.text)
            self.set_default_map_id(
                json_dict["url"].replace("/api/v1/maps/", ""))
        return response

    ## Update an existing map on the server with specified components if their
    #  paths are provided.
    # TODO: Make implementation a lot cleaner
    ## @param map_id The unique ID of the map
    ## @param map_id The document ID for the map
    ## @param map_folder The folder to look for components in
    ## @param map_image The relative filepath of the base map image file
    ## @param keepout_image The relative filepath of the keepout image file
    ## @param speedlimit_image The relative filepath of the speedlimit image
    #  file
    ## @param mapinfo Additional key-value pairs to modify in the map document
    def update_map(self,
                   map_id,
                   map_folder='',
                   map_image=None,
                   map_yaml=None,
                   keepout_image=None,
                   speedlimit_image=None,
                   set_default=False,
                   initialized=False,
                   include_default_data=True,
                   **mapinfo):
        map_id = str(map_id)
        url = self.apiurl + "maps/" + map_id
        identifier = map_id
        files = []
        iter_dict = {'map_image': map_image,
                     'keepout_image': keepout_image,
                     'speedlimit_image': speedlimit_image}
        for attach_name, relpath in iter_dict.iteritems():
            # check if entry was given
            if iter_dict[attach_name]:
                # construct full filepath
                filepath = os.path.join(map_folder, relpath)
                _, ext = os.path.splitext(filepath)
                files.append((
                    attach_name,
                    (attach_name,
                     open(filepath, self.MAP_TYPES[ext]['mode']),
                     self.MAP_TYPES[ext]['protocol'])))

        # Get metadata from yaml
        metadata = {}
        if map_yaml:
            filename = os.path.join(map_folder, map_yaml)
            with open(filename) as data:
                meta = yaml.load(data)
                metadata["resolution"] = meta["resolution"]
                metadata["x"] = meta["origin"][0]
                metadata["y"] = meta["origin"][1]
                metadata["theta"] = meta["origin"][2]
        # Apply defaults
        metadata["resolution"] = metadata.get("resolution") or 0.05
        metadata["x"] = metadata.get("x") or 0.0
        metadata["y"] = metadata.get("y") or 0.0
        metadata["theta"] = metadata.get("theta") or 0.0

        # Now upload
        map_info = {}
        # TODO: This should not be included for 99.9% of calls.
        if include_default_data:
            default_data = {
                'initialized': initialized,
                'metadata': metadata,
                'created_by': "AGENT-" + self.agentname
            }
            map_info.update(default_data)

        map_info.update(mapinfo)
        if files:
            put_data = {'data': json.dumps(map_info)}
        else:
            put_data = json.dumps(map_info)
        response = self.make_request("put", url, files=files, data=put_data)
        if set_default:
            self.set_default_map_id(identifier)
        return response

    ## Delete a map from the server
    ## @param map_id An identifier for the map
    def delete_map(self, map_id, force_default_removal=True):
        if force_default_removal:
            try:
                if map_id == self.get_default_map_id():
                    self.set_default_map_id(None)
            except ValueError:
                pass
        url = self.apiurl + "maps/" + str(map_id)
        return self.delete_document(url)

    ## Find map files from a folder and upload them.
    #  This will prioritize finding expected filenames in a desired order.
    def _get_mapfiles_from_folder(self, folder):
        # Grab the filenames in a folder
        # (see http://stackoverflow.com/a/3207973 for explanation)
        _, _, filenames = next(os.walk(folder), (None, None, []))
        files = []
        for attach_name, name in self.EXPECTED_MAP_FILES.iteritems():
            filename = os.path.join(folder, name)
            if name in filenames:
                _, ext = os.path.splitext(filename)
                files.append(
                    (attach_name,
                     (attach_name,
                      open(filename, self.MAP_TYPES[ext]['mode']),
                      self.MAP_TYPES[ext]['protocol'])))
        # Get metadata from yaml
        metadata = {}
        filename = os.path.join(folder, "map.yaml")
        if os.path.exists(filename):
            with open(filename) as data:
                meta = yaml.load(data)
                metadata["resolution"] = meta["resolution"]
                metadata["x"] = meta["origin"][0]
                metadata["y"] = meta["origin"][1]
                metadata["theta"] = meta["origin"][2]
        return files, metadata

    ## @}

    ## @name component_settings_api
    #  Clientside methods that interact with settings for agents and the system
    ## @{

    # TODO: Comments for this section

    def get_system_settings(self):
        url = self.apiurl + "settings/"
        return self.get_document(url, try_reconnect=False)

    def get_settings_for_component(self, component_name):
        url = self.apiurl + "settings/" + component_name
        return self.get_document(url, try_reconnect=False)

    def get_setting_for_component(self, component_name, name,
                                  include_timestamp=False):
        url = self.apiurl + "settings/" + component_name + "/" + name
        doc = self.get_document(url)
        try:
            if not include_timestamp:
                return doc['value']
            else:
                return doc['value'], doc['last_updated_timestamp']
        except (KeyError, TypeError):
            if not include_timestamp:
                return None
            else:
                return None, None

    def update_setting_for_component(self, component, name, value):
        url = self.apiurl + "settings/" + component + "/" + name
        return self.put_document(url, value)

    def update_settings_for_component(self, component, settings):
        url = self.apiurl + "settings/" + component
        return self.put_document(url, settings, try_reconnect=False)

    def create_settings_for_component(self, **settings):
        url = self.apiurl + "settings/"
        return self.post_document(url, settings)

    def create_setting_for_component(self, component, name, value):
        url = self.apiurl + "settings/" + component + "/" + name
        return self.post_document(url, value)

    def delete_setting_for_component(self, component, name):
        url = self.apiurl + "settings/" + component + "/" + name
        return self.delete_document(url)

    def delete_settings_for_component(self, component):
        url = self.apiurl + "settings/" + component
        return self.delete_document(url)

    ## @}

    ## @name agent_settings_api
    ## @{

    def get_own_agent_setting(self, setting):
        return self.get_agent_setting(self.agentname, setting)

    def get_agent_setting(self, agentname, setting):
        url = self.apiurl + "agents/" + agentname + "/settings/" + setting
        doc = self.get_document(url)
        try:
            return doc['value']
        except (KeyError, TypeError):
            return None

    def get_own_agent_settings(self):
        return self.get_agent_settings(self.agentname)

    def get_agent_settings(self, agentname):
        url = self.apiurl + "agents/" + agentname + "/settings"
        return self.get_document(url)

    def update_own_agent_setting(self, name, value):
        return self.update_agent_setting(self.agentname, name, value)

    def update_agent_setting(self, agentname, name, value):
        url = self.apiurl + "agents/" + agentname + "/settings/" + name
        return self.put_document(url, value)

    def update_own_agent_settings(self, **settings):
        return self.update_agent_settings(self.agentname, **settings)

    def update_agent_settings(self, agentname, **settings):
        url = self.apiurl + "agents/" + agentname + "/settings"
        return self.put_document(url, settings)

    def create_own_agent_setting(self, name, value):
        return self.create_agent_setting(self.agentname, name, value)

    def create_agent_setting(self, agentname, name, value):
        url = self.apiurl + "agents/" + agentname + "/settings/" + name
        return self.post_document(url, value)

    def delete_own_agent_setting(self, name):
        return self.delete_agent_setting(self.agentname, name,)

    def delete_agent_setting(self, agentname, name):
        url = self.apiurl + "agents/" + agentname + "/settings/" + name
        return self.delete_document(url)

    ## @}

    ## @name schedules_api
    ## @{

    def get_schedule(self, schedule_id):
        url = self.apiurl + "schedules/" + str(schedule_id)
        return self.get_document(url)

    def get_schedules(self, **params):
        url = self.apiurl + "schedules/"
        return self.get_document(url, **params)['results']

    def create_schedule(self, **schedule):
        url = self.apiurl + "schedules/"
        return self.post_document(url, schedule)

    def update_schedule(self, **schedule):
        try:
            url = self.apiurl + "schedules/" + schedule["schedule_id"]
        except KeyError:
            raise ValueError("Cannot update schedule without a schedule ID.")
        return self.put_document(url, schedule)

    def delete_schedule(self, schedule_id):
        url = self.apiurl + "schedules/" + str(schedule_id)
        return self.delete_document(url)

    ## @param task_template A valid task template ID
    ## @param schedules A list of schedule IDs to update with task_template
    ## @returns An array of responses matching **schedules**, with None for
    #           unsuccessful requests.
    def update_task_template_for_schedules(self, task_template, schedules):
        resp_array = []
        for schedule_id in schedules:
            response = None
            try:
                response = self.update_schedule(
                    **{"schedule_id": schedule_id,
                       "task_template": task_template})
            except requests.exceptions.HTTPError:
                # Don't worry about HTTP errors, just try to get everything in
                pass
            resp_array.append(response)
        return resp_array

    ## @}
