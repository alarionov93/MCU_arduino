import os
import json
from flask import Flask
from flask import render_template
from flask import request, jsonify
import models

app = Flask(__name__)

LAST_MEASURES_VALUE = 10

@app.route('/')
def index():
    measures = models.Measures.select().order_by(models.Measures.measure_id.desc())
    # measures = measures[len(measures)-3:len(measures)]
    measures_ids = []
    for m in measures:
        measures_ids.append(m.measure_id)

    without_repeates = list(set(measures_ids))
    without_repeates.reverse()
    cutted = without_repeates[:LAST_MEASURES_VALUE]
    # without_repeates.reverse()

    return render_template('index.html', measures_ids=cutted)

@app.route('/get_temperatures')
def get_temperatures():
    temperatures = models.TemperatureOut.select().order_by(models.TemperatureOut.created_at)
    temp_lst = []
    for t in temperatures:
        temp_lst.append(t.to_list())
    return jsonify(temp_lst)

@app.route('/get_measures/<int:m_id>')
def get_measures(m_id):
    if m_id is None:
        # measures = models.Measures.select().where(models.Measures.measure_id == 1).order_by(models.Measures.id)
        raise AttributeError("Measure id is None")
    else:
        measures = models.Measures.select().where(models.Measures.measure_id == int(m_id)).order_by(models.Measures.id)

    measures_lst = []
    measures_lst.append(['Id', 'Temperature Outside', 'Engine Temperature', 'Pressure', 'Voltage'])
    for m in measures:
        measures_lst.append(m.to_list())
    return jsonify(measures_lst)