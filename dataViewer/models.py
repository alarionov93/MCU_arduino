from peewee import *
import datetime


db = PostgresqlDatabase('mdu_data', user='postgres')

class BaseModel(Model):
    measure_id = IntegerField(unique=False, null=True, default=0)

    class Meta:
        database = db

def get_measure_id(table):
    # TODO: think how divide measurements !!
    if not table:
        return ProgrammingError('Not found data source for getting last measure id')
    cursor = db.execute_sql('SELECT MAX(measure_id) FROM %;' % table)
    res = cursor.fetchone()
    m_id = int(res)
    return m_id

class TemperatureOut(BaseModel):

    created_at = DateTimeField(default=datetime.datetime.now)
    comment = CharField(unique=False, null=True, default='')
    value = FloatField(unique=False, null=False, default=0.0)

    def to_json(self):
        return {
            'id': self.id,
            'measure_id': self.measure_id,
            'created_at': self.created_at,
            'value': self.value
        }

    def to_list(self):
        return [
            self.created_at,
            self.value
        ]


# class TemperatureEng(BaseModel):
#     created_at = DateTimeField(default=datetime.datetime.now)
#     comment = CharField(unique=False, null=True, default='')
#     value = FloatField(unique=False, null=False, default=0.0)
#
#     def to_json(self):
#         return {
#             'id': self.id,
#             'measure_id': self.measure_id,
#             'created_at': self.created_at,
#             'value': self.value
#         }
#
#     def to_list(self):
#         return [
#             self.created_at,
#             self.value
#         ]

class Measures(BaseModel):
    temp_out = FloatField(unique=False, null=False, default=0.0)
    temp_eng = FloatField(unique=False, null=False, default=0.0)
    pressure = FloatField(unique=False, null=False, default=0.0)
    voltage = FloatField(unique=False, null=False, default=0.0)
    gps_charge_stat = IntegerField(unique=False, null=True, default=0)
    gps_charge_val = IntegerField(unique=False, null=True, default=0)
    created_at = DateTimeField(default=datetime.datetime.now)

    def tracker_chg_info_to_list(self):
        return [
            self.id,
            self.gps_charge_stat,
            self.gps_charge_val
        ]

    def to_list(self):
        return [
            self.id,
            self.temp_out,
            self.temp_eng,
            self.pressure,
            self.voltage
        ]