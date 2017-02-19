import models

models.db.connect()

# try:
#     print("Try to remove table Temperature...")
#     models.db.drop_table(models.Temperature)
# except:
#     print("Table Temperature does not exists!")
# finally:
print("Try to create table Temperature...")
models.db.create_table(models.TemperatureOut)

print("Try to create table Measures...")
models.db.create_table(models.Measures)

# try:
#     print("Try to remove table EventGroup...")
#     models.db.drop_table(models.EventGroup)
# except Exception as e:
#     print(e)
# finally:
#     print("Try to create table EventGroup...")
#     models.db.create_table(models.EventGroup)


models.db.close()