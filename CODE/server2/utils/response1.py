from flask import jsonify

def create_response(msg, error=False):
    d = dict()
    if error == True:
        # error msg
        d = {
            "StatusCode":"?",
            "StatusMsg":"?",
            "Msg":msg
        }
    else:
        # normal msg
        d = {
            "StatusCode":"200",
            "StatusMsg":"OK",
            "Msg":msg
        }

    return jsonify(d)