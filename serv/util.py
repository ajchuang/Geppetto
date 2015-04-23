
def read_conf (idx):
    with open ('config') as f:
        for line in f:
            toks = line.split ()

            if toks[0] == idx:
                return [toks[1], int(toks[2])]
