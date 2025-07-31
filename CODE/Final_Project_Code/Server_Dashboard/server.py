from flask import Flask, request, jsonify, render_template
from utilities.modules import execute_query, execute_select_query

server = Flask(__name__, template_folder='templates')

@server.route('/')
def home():
    return render_template('index.html')  # Serves the dashboard HTML

@server.route('/sensor', methods=['POST', 'GET'])
def handle_sensor_data():
    if request.method == 'POST':
        data = request.get_json()

        ldr = data.get('LDR_DATA')
        mq5 = data.get('MQ5_DATA')
        temp = data.get('TEMPERATURE')
        hum = data.get('HUMIDITY')
        loc = data.get('LOCATION')

        if None in [ldr, mq5, temp, hum, loc]:
            return jsonify({"error": "Missing one or more sensor fields"}), 400

        query = f" INSERT INTO SensorData (LDR_DATA, MQ5_DATA, TEMPERATURE, HUMIDITY, LOCATION) VALUES ({ldr}, {mq5}, {temp}, {hum}, '{loc}'); "
        execute_query(query)
        return jsonify({"message": "Sensor data inserted successfully"}), 201

    elif request.method == 'GET':
        query = "SELECT * FROM SensorData ORDER BY DATE_TIME DESC;"
        data = execute_select_query(query)
        return jsonify(data), 200

if __name__ == '__main__':
    server.run(host='0.0.0.0', port=5000, debug=True)
