import datetime

with open('temp.txt', "r") as f:
        pure_data = f.read()
        splitted_data = pure_data.split('\n')
        print(splitted_data)
        data_lst = []
        for d in splitted_data:
        	data_lst.append([
        		str(datetime.datetime.now()), d.split("=")[0]
        	])
        print(data_lst)
# with open('temp.json', "a") as f1:
	# f1.write(json.dumps(data_lst))