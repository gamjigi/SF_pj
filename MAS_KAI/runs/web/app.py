from flask import Flask, request, jsonify, render_template
import mysql.connector

app = Flask(__name__)

def connect_db():
    return mysql.connector.connect(
        host='localhost',
        user='root',
        password='1234',
        database='webtest'
    )

@app.route('/')
def index():
    return render_template('test.html')

@app.route('/submit', methods=['POST'])
def submit():
    data = request.json
    conn = connect_db()
    cursor = conn.cursor()
    query = """
    INSERT INTO new_table (name, age, symptoms, birthdate, room_number, gender)
    VALUES (%s, %s, %s, %s, %s, %s)
    """
    cursor.execute(query, (
        data['name'],
        data['age'],
        data['symptoms'],
        data['birthdate'],
        data['room_number'],
        data['gender']
    ))
    conn.commit()
    cursor.close()
    conn.close()
    return jsonify({"status": "success"}), 201

@app.route('/reset', methods=['POST'])
def reset():
    conn = connect_db()
    cursor = conn.cursor()
    query = "TRUNCATE TABLE new_table"  # 테이블 내용을 초기화하는 SQL 쿼리
    cursor.execute(query)
    conn.commit()
    cursor.close()
    conn.close()
    return jsonify({"status": "success"}), 200

@app.route('/data', methods=['GET'])
def get_data():
    conn = connect_db()
    cursor = conn.cursor()
    cursor.execute("SELECT name, age, symptoms, birthdate, room_number, gender FROM new_table")
    rows = cursor.fetchall()
    cursor.close()
    conn.close()
    return jsonify(rows)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
