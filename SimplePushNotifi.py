# A file pushbullet.sh with execute permission in the
# directory /usr/bin is a prerequisite

import os

sec = 6
string = '/usr/bin/pushbullet.sh ' + str(sec)
os.system(string)
