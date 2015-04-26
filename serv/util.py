import datetime

# time utilities
def unix_now ():
    now = datetime.datetime.now ()
    return unix_time_millis (now)

def unix_time(dt):
    epoch = datetime.datetime.utcfromtimestamp(0)
    delta = dt - epoch
    return delta.total_seconds()

def unix_time_millis(dt):
    return unix_time(dt) * 1000.0

# configuration functions
def read_conf (idx):
    with open ('config') as f:
        for line in f:
            toks = line.split ()

            if toks[0] == idx:
                return [toks[1], int(toks[2]), int(toks[3])]
    return [None, None, None]


def read_all_conf ():

    g_dic = {}
    
    with open ('config') as f:
        for line in f:

            line.strip ()
            
            if line[0] == '#':
                continue

            toks = line.split ()
            
            # bad lines
            if len (toks) != 4:
                print '[Error] config is incorrect: ' + line
                continue;

            # add to dictionary
            g_dic[toks[0]] = (toks[1], toks[2], toks[3]);

    return g_dic
