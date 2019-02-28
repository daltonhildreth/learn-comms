import numpy

def read_col(f, c):
    f = open(f, "r")
    f = f.read()
    f = [l.split('\t') for l in f.split('\n')]
    f = [l[c] for l in f[:-1]]
    f = [float(i.split(' ')[2]) for i in f]
    return numpy.array(f)

def make_data_t():
    datasets = [
        "feb26_1249_local_normrel", "feb26_1408_control",
        "feb27_1158_local", "feb27_1354_local_dot",
        "feb27_1520_local_inv_dist", "feb27_1638_vel",
        "feb27_1929_l_dot_prox", "feb27_2137_2latent",
        "feb27_2306_lc2_softsign_goal", "feb28_0340_lc2_dp_sg"
    ]
    datasets = ["data/"+f+"/results/best.txt" for f in datasets] 
    datasets = [(f.split('/')[1],read_col(f,2),read_col(f,3)) for f in datasets]
    data_t = list(zip(*datasets))
    print(max((numpy.mean(a) for a in data_t[2])))
    print([numpy.mean(a) for a in data_t[2]])
    print(data_t[0])
    b = data_t[2][7]
    print(b)
    pd = [i/b for i in data_t[1]]
    print(pd)
    pdu = [numpy.mean(i) for i in pd]
    print(pdu)
    zpdu = list(zip(data_t[0], pdu))
    zpdu = [(j,i) for i,j in zpdu]
    zpdu.sort()
    for i in zpdu:
        print(i)
    return data_t, zpdu, pdu, pd, b, datasets
