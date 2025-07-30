from flask import Flask, request
from utils.database import execute_query

app = Flask(__name__)

@app.route('/sensordata', methods=['POST'])
def insert_sensor_data():
    data = request.get_json()

    MQ5 = data.get('MQ5_DATA')
    Temperature = data.get('TEMPERATURE')
    Humidity = data.get('HUMIDITY')
    LDR = data.get('LDR_DATA')
    Location = data.get('LOCATION')

    query = f"""
        INSERT INTO Sensors_Data (MQ5, Temperature, Humidity, LDR, LOCATION)
        VALUES ({MQ5}, {Temperature}, {Humidity}, {LDR}, '{Location}');
    """

    execute_query(query)
    return "Sensor data inserted successfully"

@app.route('/sensordata', methods=['GET'])
def get_all_data():
    from utils.database import execute_select_query
    query = "SELECT * FROM Sensors_Data;"
    return execute_select_query(query)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=4000, debug=True)
