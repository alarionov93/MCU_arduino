import datetime
import json
import models
import sys

# TODO: create flask app with database and load data into it

DEFAULT_LOG_FILENAME = 'mdu.log'
cursor = models.db.execute_sql('SELECT MAX(measure_id) FROM measures;')
res = cursor.fetchone()
print(res)
new_measure_id = None
if res[0] is not None:
    last_measure_id = int(res[0])
    if len(sys.argv) > 1:
        if sys.argv[1] == "--new":
            new_measure_id = last_measure_id + 1
else:
    last_measure_id = 0

try:
    with open(DEFAULT_LOG_FILENAME, "r") as f:
        import pdb
        pure_data = f.read()
        splitted_data = pure_data.split('NA;')
        for data in splitted_data:
            # temp = models.TemperatureOut(value=float(d.split("=")[1].split(";")[0]))
            # temp.save()
            # print(data)
            p_idx = data.find("P=")
            # p_delimiter_idx = data.find(";", p_idx)
            v_idx = data.find("V=")
            # v_delimiter_idx = data.find(";", v_idx)
            t_out_idx = data.find("T2=")
            # t_out_delimiter_idx = data.find(";", t_out_idx)
            t_eng_idx = data.find("T1=")
            ch_stat_idx = data.find("CHG=")
            ch_val_idx = data.find("CH=")
            # t_eng_delimiter_idx = data.find(";", t_eng_idx)
            if all([p_idx, v_idx, t_eng_idx, t_out_idx, ch_stat_idx, ch_val_idx]):
                # p_val = data[p_idx + 2 : p_delimiter_idx]
                # v_val = data[v_idx + 2 : v_delimiter_idx]
                # t_out_val = data[t_out_idx + 3 : t_out_delimiter_idx]
                # t_eng_val = data[t_eng_idx + 3 : t_eng_delimiter_idx]
                #
                # print(p_val)
                # print(v_val)
                # print(t_out_val)
                # print(t_eng_val)
                splitted = data.split("\n")
                for s in splitted:
                    if len(s) < 1:
                        splitted.remove(s)
                if len(splitted) < 6:
                    print("error in data!\n")
                else:
                    # print(splitted)
                    # TODO: VERY IMPORTANT !!! SEND CHG DATA FROM GPS TRACKER AT THE END OF MAIN LOOP !!!
                    # p_val = splitted[0][2:len(splitted[0])-1]
                    # v_val = splitted[1][2:len(splitted[1])-1]
                    # t_eng_val = splitted[2][3:len(splitted[2])-1]
                    # t_out_val = splitted[3][3:len(splitted[3])-1]
                    # ch_stat_val = splitted[4][4:len(splitted[4])-1]
                    # ch_val = splitted[5][3:len(splitted[5])-1]
                    for s in splitted:
                        ch_stat_val = None
                        ch_val = None
                        if s.find("P=") != -1:
                            p_val = s[2:len(s)-1]
                            # print(p_val)
                        elif s.find("V=") != -1:
                            v_val = s[2:len(s)-1]
                            # print(v_val)
                        elif s.find("CH=") != -1:
                            ch_stat_val = s[3:len(s)-1]
                            # print(ch_stat_val)
                        elif s.find("CHG=") != -1:
                            ch_val = s[4:len(s)-1]
                            # print(ch_val)
                        elif s.find("T1=") != -1:
                            t_eng_val = s[3:len(s)-1]
                            # print(t_eng_val)
                        elif s.find("T2=") != -1:
                            t_out_val = s[3:len(s)-1]
                            # print(t_out_val)
                        else:
                            print("no data found")
                    if all([p_val, v_val, t_eng_val, t_out_val]):
                        # print("all values catched")
                        measure = models.Measures(temp_out=float(t_out_val))
                        if new_measure_id:
                            measure.measure_id = new_measure_id
                        elif last_measure_id:
                            measure.measure_id = last_measure_id
                        else:
                            print("last or new measure id is not defined")
                        measure.temp_eng = float(t_eng_val)
                        measure.voltage = float(v_val)
                        measure.pressure = float(p_val)
                        if ch_stat_val:
                            measure.gps_charge_stat = int(ch_stat_val)
                        if ch_val:
                            measure.gps_charge_val = int(ch_val)
                        measure.save()
                        print("stored to db")
            else:
                print("missing data!\n")


    # with open('data/temp2.txt', "r") as f:
    #     pure_temp_eng = f.read()
    #     temp_eng_lst = []
    #     splitted_temp_eng = pure_temp_eng.split('\n')
    #     for d in splitted_temp_eng:
    #         temp_eng_lst.append(float(d.split("=")[1].split(";")[0]))
    #         # temp = models.TemperatureOut(value=float(d.split("=")[1].split(";")[0]))
    #         # temp.save()
    #     data_lst.append(temp_eng_lst)

    # with open('data/pressure.txt', "r") as f:
    #     import pdb
    #     pure_pressure = f.read()
    #     splitted_pressure = pure_pressure.split('\n')
    #
    # with open('data/voltage.txt', "r") as f:
    #     pure_voltage = f.read()
    #     splitted_voltage = pure_voltage.split('\n')
except Exception as e:
    print(e)

# for val_lst in data_lst:
#     for d in val_lst:
#         measure = models.Measures(temp_out=float(d.split("=")[1].split(";")[0]))
#         measure.temp_eng = 0
#         measure.voltage = 0
#         measure.pressure = 0
#
#     measure.save()

# for d in splitted_data:
#     measure = models.Measures(float(d.split("=")[1].split(";")[0]))
#     measure.save()

# with open('data/temp1.json', "a") as f1:
#     f1.write(json.dumps(data_lst))

# with open('data/temp2.txt', "r") as f:
#         import pdb
#         pure_data = f.read()
#         splitted_data = pure_data.split('\n')
#         # print(splitted_data)
#         data_lst = []
#         for d in splitted_data:
#             data_lst.append([
#                 str(datetime.datetime.now()), float(d.split("=")[1].split(";")[0])
#             ])
#             # pdb.set_trace()
#         print(data_lst)
#
# with open('data/temp2.json', "a") as f1:
#     f1.write(json.dumps(data_lst))
#
# with open('data/pressure.txt', "r") as f:
#         import pdb
#         pure_data = f.read()
#         splitted_data = pure_data.split('\n')
#         # print(splitted_data)
#         data_lst = []
#         for d in splitted_data:
#             data_lst.append([
#                 str(datetime.datetime.now()), float(d.split("=")[1].split(";")[0])
#             ])
#             # pdb.set_trace()
#         print(data_lst)
#
# with open('data/pressure.json', "a") as f1:
#     f1.write(json.dumps(data_lst))
#
# with open('data/voltage.txt', "r") as f:
#         import pdb
#         pure_data = f.read()
#         splitted_data = pure_data.split('\n')
#         # print(splitted_data)
#         data_lst = []
#         for d in splitted_data:
#             data_lst.append([
#                 str(datetime.datetime.now()), float(d.split("=")[1].split(";")[0])
#             ])
#             # pdb.set_trace()
#         print(data_lst)
#
# with open('data/voltage.json', "a") as f1:
#     f1.write(json.dumps(data_lst))
#
# with open('data/gps_voltage.txt', "r") as f:
#         import pdb
#         pure_data = f.read()
#         splitted_data = pure_data.split('\n')
#         # print(splitted_data)
#         data_lst = []
#         for d in splitted_data:
#             data_lst.append([
#                 str(datetime.datetime.now()), float(d.split("=")[1].split(";")[0])
#             ])
#             # pdb.set_trace()
#         print(data_lst)
#
# with open('data/gps_voltage.json', "a") as f1:
#     f1.write(json.dumps(data_lst))

