import csv
import object_detector
import rospkg
import bibs_extractor

class csv_reader:
    def __init__(self):
        self.bibs_enemy = None
        self.bibs_friend = None

    def read(self):
        rospack = rospkg.RosPack()
        f = open(rospack.get_path('robocup_object_detector')+'/params/bibs_data.csv', 'rb')
        #f = open('../params/bibs_data.csv', 'rb')
        dataReader = csv.reader(f)
        for row in dataReader:
            #check friend
            if row[4] == "T":
                self.bibs_friend = bibs_extractor.bibs(row[0],float(row[2])*360,float(row[3])*360,float(row[1]),True)
            #check enemy
            if row[5] == "T":
                self.bibs_enemy = bibs_extractor.bibs(row[0],float(row[2])*360,float(row[3])*360,float(row[1]),False)
        if self.bibs_enemy is not None:
            if self.bibs_friend is not None:
                return True
        f.close()
        return False


if __name__ == '__main__':
    reader = csv_reader()
    print reader.read()
