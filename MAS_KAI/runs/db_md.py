import mysql.connector

def fetch_data_from_db():
    connection = mysql.connector.connect(
        host='localhost',
        user='root',
        password='1234',
        database='information'
    )

    cursor = connection.cursor()
    query = "SELECT * FROM info"
    cursor.execute(query)

    results = cursor.fetchall()
    column_names = [desc[0] for desc in cursor.description]

    cursor.close()
    connection.close()

    return results, column_names
