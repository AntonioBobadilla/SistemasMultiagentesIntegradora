import flask
from flask.json import jsonify
import uuid
import robots
from robots import Maze, Robot, Box
games = {}

app = flask.Flask(__name__)

@app.route("/games", methods=["POST"])
def create():
    global games
    id = str(uuid.uuid4())
    games[id] = Maze()
    return "ok", 201, {'Location': f"/games/{id}"}


@app.route("/games/<id>", methods=["GET"])
def queryState(id):
    global model
    model = games[id]
    model.step()
    listaRobots = []
    #buscar todos los agentes del modelo y si son cajas o robots incluirlos en el arreglo de diccionarios a mandar al json
    for i in range(len(model.schedule.agents)):
        agent = model.schedule.agents[i]
        if type(agent) is robots.Robot:
            listaRobots.append({"x": agent.pos[0], "y": agent.pos[1], "tipo" : "Robot", "cajasRecogidas": agent.inventory, "leavingBoxes": agent.leavingBoxes})
        elif type(agent) is robots.Box:
            listaRobots.append({"x": agent.pos[0], "y": agent.pos[1], "tipo" : "Caja", "limpio": agent.limpio, "stackAsignadoX": agent.stackAsignadoX, "stackAsignadoY": agent.stackAsignadoY, "inStack": agent.inStack})
        elif type(agent) is robots.Stack:
            listaRobots.append({"x": agent.pos[0], "y": agent.pos[1], "tipo" : "Stack", "boxesInStack": agent.cajas})
        else:
            i = i - 1
    return jsonify({"Items":listaRobots})

app.run(debug = True)