# # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Xbox Controller Button List:                        #
# - - - - - - - - - - - - - - - - - - - - - - - - - - #
# Axis:                                               #
# 0.- LS left, right   # 1.- LS up, down   # 2.- LT   #
# 3.- RS left, right   # 4.- RS up, down   # 5.- RT   #
#                                                     #
# Buttons:                                            #
# 0.- A   # 5.- RB     # 10.- RS                      #
# 1.- B   # 6.- Back   # 11.- Left                    #
# 2.- X   # 7.- Start  # 12.- Right                   #
# 3.- Y   # 8.- Xbox   # 13.- Up                      #
# 4.- LB  # 9.- LS     # 14.- Down                    #
# # # # # # # # # # # # # # # # # # # # # # # # # # # #
import subprocess

class KeyMapper(object):

    def __init__(self):
        print(self.is_original())

        if self.is_original():

            # - - - - BUTTON DEFINITIONS - - - -
            self.b = {
                'A'     : 0,
                'B'     : 1,
                'X'     : 2,
                'Y'     : 3,
                'LB'    : 4,
                'RB'    : 5,
                'BACK'  : 6,
                'START' : 7,
                'XBOX'  : 8,
                'LS'    : 9,
                'RS'    : 10,
                'LEFT'  : 11,
                'RIGHT' : 12,
                'UP'    : 13,
                'DOWN'  : 14
            }

            # - - - - AXES NUMBERS - - - -
            self.a = {
                'LS_HORZ' : 0,
                'LS_VERT' : 1,
                'LT'      : 2,
                'RS_HORZ' : 3,
                'RS_VERT' : 4,
                'RT'      : 5
            }
        else:
            # - - - - BUTTON DEFINITIONS - - - -
            self.b = {
                'Y'     : 0,
                'B'     : 1,
                'A'     : 2,
                'X'     : 3,
                'LB'    : 4,
                'RB'    : 5,
                'LT'    : 6,
                'RT'    : 7,
                'BACK'  : 8,
                'START' : 9,
                'XBOX'  : 10,
                'L3'    : 11,
                'R3'    : 12,
            }
            # - - - - AXES NUMBERS - - - -
            self.a = {
                'LS_HORZ' : 0,
                'LS_VERT' : 1,
                'RS_HORZ' : 2,
                'RS_VERT' : 3,
                'RIGHT'   : 4,
                'LEFT'    : 4,
                'UP'      : 5,
                'DOWN'    : 5,   
            }


    def has_button(self, name):
        return name in self.b

    def has_axis(self, name):
        return name in self.a

    def get_button_id(self, name):
        if not name in self.b:
            print("unknown button named: %s" % name)
            return None
        return self.b[name]

    def get_button_ids(self, names):
        ids = []
        for name in names:
            ids.append(self.get_button_id(name))
        return ids

    def get_axis_id(self, name):
        if not name in self.a:
            print("unknown axis named: %s" % name)
            return None
        return self.a[name]
    
    def is_original(self):
        df = subprocess.check_output("lsusb")
        return "Xbox" in df

