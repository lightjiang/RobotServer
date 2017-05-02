#!/usr/bin/python

## @file settings.py fetchcore settings for client use

import os

# For local testing, CORE_SERVER_URL should be set to localhost.
# For system testing this CORE_SERVER_URL should be set to fetchtest.
# For continuous integration, fetchcore should map to localhost

TIMESTAMP_FORMAT = "%Y-%m-%d %H:%M:%S.%f"
CORE_SERVER_URL = os.getenv("CORE_SERVER_URL",
                            "http://fetchcore.local:9000")

TEST_CORE_SERVER_URL = os.getenv("TEST_CORE_SERVER_URL",
                                 "http://localhost:9090")
