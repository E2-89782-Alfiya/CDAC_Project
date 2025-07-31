import mysql.connector

def execute_query(query):
    # create connection 
    conn = mysql.connector.connect(
        host='localhost',
        port=3306,
        user="root",
        password="root@2001",
        database="iotdb"
    )

    # create cursor to execute query
    cursor = conn.cursor()

    # execute a query with cursor
    cursor.execute(query)

    # commit all changes on mysql server
    conn.commit()

    # close the cursor
    cursor.close()

    # close the connection
    conn.close()


def execute_select_query(query):
    # create connection 
    conn = mysql.connector.connect(
        host='localhost',
        port=3306,
        user="root",
        password="root@2001",
        database="iotdb"
    )

    # create cursor to execute query
    cursor = conn.cursor()

    # execute a query with cursor
    cursor.execute(query)

    # fetch all data from cursor
    data = cursor.fetchall()

    # close the cursor
    cursor.close()

    # close the connection
    conn.close()

    # return data
    return data
