
def read_conf (idx):
    with open ('config') as f:
        for line in f:
            toks = line.split ()

            if toks[0] == idx:
                return [toks[1], int(toks[2])]

def read_all_conf ():
    dic = {}

    with open ('config') as f:
        for line in f:
            toks = line.split ()
            
            # bad lines
            if len (toks) != 3:
                print 'I can not parse: ' + line
                continue;

            # add to dictionary
            dic[toks[0]] = (toks[1], toks[2]);

    return dic
