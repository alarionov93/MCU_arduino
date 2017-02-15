import datetime
import json

with open('data/temp1.txt', "r") as f:
        import pdb
        pure_data = f.read()
        splitted_data = pure_data.split('\n')
        # print(splitted_data)
        data_lst = []
        for d in splitted_data:
            data_lst.append([
                str(datetime.datetime.now()), float(d.split("=")[1].split("C")[0])
            ])
            # pdb.set_trace()
        print(data_lst)

with open('data/temp1.json', "a") as f1:
    f1.write(json.dumps(data_lst))

with open('data/temp2.txt', "r") as f:
        import pdb
        pure_data = f.read()
        splitted_data = pure_data.split('\n')
        # print(splitted_data)
        data_lst = []
        for d in splitted_data:
            data_lst.append([
                str(datetime.datetime.now()), float(d.split("=")[1].split("C")[0])
            ])
            # pdb.set_trace()
        print(data_lst)

with open('data/temp2.json', "a") as f1:
    f1.write(json.dumps(data_lst))

with open('data/pressure.txt', "r") as f:
        import pdb
        pure_data = f.read()
        splitted_data = pure_data.split('\n')
        # print(splitted_data)
        data_lst = []
        for d in splitted_data:
            data_lst.append([
                str(datetime.datetime.now()), float(d.split("=")[1].split("psi")[0])
            ])
            # pdb.set_trace()
        print(data_lst)

with open('data/pressure.json', "a") as f1:
    f1.write(json.dumps(data_lst))

with open('data/voltage.txt', "r") as f:
        import pdb
        pure_data = f.read()
        splitted_data = pure_data.split('\n')
        # print(splitted_data)
        data_lst = []
        for d in splitted_data:
            data_lst.append([
                str(datetime.datetime.now()), float(d.split("=")[1].split("V")[0])
            ])
            # pdb.set_trace()
        print(data_lst)

with open('data/voltage.json', "a") as f1:
    f1.write(json.dumps(data_lst))