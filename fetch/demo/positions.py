import csv


class Positionval:
    sr = 0
    position_id = ''
    location_name = ''
    position = ''
    selected = 0
    quantity = 0
    status = 0

    def __init__(self, obj):
        self.sr, self.position_id, self.location_name, self.position, self.selected, self.quantity, self.status = obj

class Positions:
    data = []

    def __init__(self, filename):
        with open(filename) as f:
            f = csv.reader(f)
            next(f)

            for row in f:
                xpos = Positionval(row)
                self.data.append(xpos)

        #self.data.__len__()
        #print self.data[0].position_id
if __name__ == "__main__":
    obj = Positions('positions.csv')
    print obj.data[0].sr
    print obj.data[0].position_id
    print obj.data[0].location_name
    print obj.data[0].position
    print obj.data[0].selected
    print obj.data[0].quantity
    print obj.data[0].status
