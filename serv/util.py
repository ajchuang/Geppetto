g_dic = {}

def read_conf (idx):
    with open ('config') as f:
        for line in f:
            toks = line.split ()

            if toks[0] == idx:
                return [toks[1], int(toks[2])]

def read_all_conf ():

    with open ('config') as f:
        for line in f:
            toks = line.split ()
            
            # bad lines
            if len (toks) != 4:
                print '[Error] The file format is incorrect: ' + line
                continue;

            # add to dictionary
            g_dic[toks[0]] = (toks[1], toks[2], toks[3]);

    return g_dic
