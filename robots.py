#Programa que crea una simulacion donde robots limpian las diferentes casillas que existen en un tablero. En caso de que el robot se encuentre en una casilla
#limpia, tendra que moverse, de lo contrario tendra que limpiarla antes de moverse. El programa recibe NxM filas y columnas del tablero respectivamente, asi como
#el tiempo de ejecucion del programa (numero de movimientos posibles para los robots)y la cantidad de bloques sucios que se encuentran al inicio de la ejecucion.

#Realizado por:
#David Zárate López A01329785
#Karen Rugerio Armenta  A01733228
#José Antonio Bobadilla García A01734433
#Ultima modificacion: 11/11/2021


from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from mesa.space import MultiGrid
from mesa.visualization.UserParam import UserSettableParameter
from mesa.datacollection import DataCollector
from mesa.visualization.modules import ChartModule

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid as PathGrid
from pathfinding.finder.a_star import AStarFinder


#Agente robot
class Robot(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    #se asigna el tipo robot a la clase
    self.type = "robot"
    #movimientos realizados por el robot
    self.movements = 0
    #numero de casillas limpiadas por el robot
    self.limpiadas = 0
    #numero de cajas que tiene el robot
    self.inventory = 0
    # indice auxiliar
    self.index = 0
   # stack al que tiene que dejar las cajas
    self.allocatedStack = 0
  # Atributo para dejar cajas despues
    self.empty = True
  # Atributo que nos servira en la api para dejar cajas
    self.leavingBoxes = False

  def step(self):
    #se elige al azar una posicion para que el robot avance
    next_moves = self.model.grid.get_neighborhood(self.pos, moore=True)
    next_move = self.random.choice(next_moves)
    # el robot revisa si ya tiene 5 cajas
    full = self.checkIfFull()

    if not full:
      self.leavingBoxes = False
      #el robot revisa si el bloque en el que esta se encuentra limpio
      clean = self.pickup()
      #si el robot todavia puede hacer movimientos y la casilla esta limpia significa que puede moverse
      if self.movements >= self.model.time:
        print("Se ha acabado el tiempo de ejecucion , teniendo un total de",  self.movements * self.model.robots,"movimientos")
      if (self.model.Box == 0 and self.allRobotsEmpty()): # te quedaste probando esto
        print("se han recogido todas las cajas en un total de", self.getTotalMovements(),"movimientos")
      if(self.movements < self.model.time and clean):
        self.movements += 1
        self.model.grid.move_agent(self, next_move)
    else:
      

      path = self.getPathToClosestStack(self.allocatedStack)

      if(self.index < len(path)): # si entra sigue en camino
        next_move = path[self.index]
        self.index = 1
        self.model.grid.move_agent(self, next_move)
      else: # si entra ya llegó a stack
        self.index = 0
        self.inventory = 0
        self.leaveBoxes()
        self.leavingBoxes = True
        self.assignNewStack()
      clean = True
 
    #funcion auxiliar que dirige el robot hacia el stack mas cercano    
  def getTotalMovements(self):
    movements = 0
    for robot in self.model.arrRobots:
      movements += robot.movements
    return movements

    #funcion auxiliar que dirige el robot hacia el stack mas cercano    
  def allRobotsEmpty(self):
    for robot in self.model.arrRobots:
        if not robot.empty:
          return False
    return True    

   #funcion auxiliar que dirige el robot hacia el stack mas cercano    
  def assignNewStack(self):
    if (len(self.model.stacksPos) > 0):
         
      stackIndex = self.random.randint(0, len(self.model.stacksPos)-1)
      tupleOfStack = self.model.stacksPos[stackIndex]
      # asignamos el siguiente movimiento como el stack
      self.allocatedStack = tupleOfStack
      del self.model.stacksPos[stackIndex]

   #funcion auxiliar que dirige el robot hacia el stack mas cercano    
  def leaveBoxes(self):
    cellmates = self.model.grid.get_cell_list_contents([self.pos])
    for block in cellmates:
      if(block.type == "Stack" ):
        block.cajas = 5
        self.inventory = 0
        self.index = 0 

   #funcion auxiliar que saca la ruta hacia el stack mas cercano
  def getPathToClosestStack(self, path):
      grid = PathGrid(matrix=self.model.matrix)
      grid.cleanup()
      start = grid.node(self.pos[0],self.pos[1])
      end = grid.node(self.allocatedStack[0],self.allocatedStack[1])
      finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
      path, runs = finder.find_path(start, end, grid)
      return path

   #funcion auxiliar que saca la ruta hacia el stack mas cercano
  def getPathToEmptyStack(self, stack):
      grid = PathGrid(matrix=self.model.matrix)
      grid.cleanup()
      start = grid.node(self.pos[0],self.pos[1])
      end = grid.node(stack.pos[0],stack.pos[1])
      finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
      path, runs = finder.find_path(start, end, grid)
      return path

  #funcion auxiliar que revisa si el robot ya tiene 5 cajas
  def checkIfFull(self):
    if self.inventory == 5:
      return True
    else:
      return False

  def createArrayOfNotFullStacks(self):
    index = 0
    for stack in self.model.arrStacks:
        if stack.full:
          del self.model.arrStacks[index]
        index += 1

  def leaveMissingBoxes(self):
    cellmates = self.model.grid.get_cell_list_contents([self.pos])
    for block in cellmates:
      if(block.type == "Stack" ):
       if (block.cajas < 5):
         if ((block.cajas + self.inventory) < 5 ):
           block.cajas = block.cajas + self.inventory
           self.empty = True
           self.inventory = 0
         elif (((block.cajas + self.inventory) == 5 )):
            block.cajas = 5
            self.inventory = 0
            block.full = True          
         else: 
           spareBoxes = (block.cajas + self.inventory) - 5
           block.cajas = 5
           block.full = True
           self.inventory = spareBoxes 
           

  def goToEmptyStack(self):
    if (len(self.model.arrStacks) > 0):
      path = self.getPathToEmptyStack(self.model.arrStacks[0])
      if(self.index < len(path)): # si entra sigue en camino
        next_move = path[self.index]
        self.index += 1
        self.model.grid.move_agent(self, next_move)
      else: # si entra ya llegó a stack
        self.index = 0
        self.leaveMissingBoxes()

  #funcion auxiliar que revisa si la casilla donde se encuentra el robot esta limpia
  def pickup(self):
    cellmates = self.model.grid.get_cell_list_contents([self.pos])
    for block in cellmates:
      if self.model.Box == 0:
       if self.inventory == 0:
          self.empty = True
          return False # que dejen de moverse
       elif self.inventory != 0:
          self.createArrayOfNotFullStacks()
          self.goToEmptyStack()
      if(block.type == "Box" ):
        block.limpio = True
        block.stackAsignadoX = self.allocatedStack[0]
        block.stackAsignadoY = self.allocatedStack[1]
        block.changeColor()
        self.limpiadas += 1
        self.inventory += 1
        block.inStack = self.inventory
        self.model.Box -= 1
        return False
      else:
      #elif(block.type == "CleanBlock"):
        return True



#Agente casilla Sucia
class Box(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    self.type = "Box"
    self.limpio = False
    self.stackAsignadoX = -1
    self.stackAsignadoY = -1
    self.inStack = -1

  #funcion  que cambia de color a la casilla al cambiarla de tipo sucia a limpia
  def changeColor(self):
    self.type = "CleanBlock"

#Agente casilla limpia
class CleanBlock(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    self.type = "CleanBlock"
    self.limpio = True

#Agente Stacks
class Stack(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    self.type = "Stack"
    self.cajas = 0
    self.full = False


class Maze(Model):
  #se asignan las variables modificables por el usuario siendo filas, columnas, robots, tiempo de ejecucion y el numero de bloques sucios
  def __init__(self, rows =10,columns = 10, robots = 5, time = 20000, Box = 60):
    super().__init__()
    self.schedule = RandomActivation(self)
    self.rows = rows
    self.columns = columns
    self.robots = robots
    self.time = time
    self.Box = Box
    self.Stacks = (Box // 5)
    #if self.Stacks % 5 > 0:
      #self.Stacks += 1
    self.grid = MultiGrid(self.columns, self.rows, torus=False)
    self.matrix = []
    self.stacksPos = []
    self.arrStacks = []
    self.arrRobots = []
    #se crea una matriz de ceros para identificar las casillas sucias y limpias
    self.createCeroMatrix()
    #se crean los robots y bloques tanto limpios como sucios para colocarse en el tablero
    self.placeStacks()
    self.placeRobots()
    self.placeBox()
    self.placeCleanBlocks()
 

  def step(self):
    self.schedule.step()
  
  @staticmethod
  def count_type(model):
      return model.Box

  #se crean los bloques sucios de manera aleatoria
  def placeBox(self):
    blocks = self.Box
    while blocks > 0:
      randomX = self.random.randint(0, self.rows-1)
      randomY = self.random.randint(0, self.columns-1)
      while self.matrix[randomX][randomY] == 1:
        randomX = self.random.randint(0, self.rows-1)
        randomY = self.random.randint(0, self.columns-1)
      block = Box(self,(randomY,randomX))
      self.grid.place_agent(block, block.pos)
      self.matrix[randomX][randomY] = 1
      blocks -= 1
      self.schedule.add(block)

  #se crean los bloques limpios
  def placeCleanBlocks(self):
     for _,x,y in self.grid.coord_iter():
      if self.matrix[y][x] == 0:
        block = CleanBlock(self,(x,y))
        self.grid.place_agent(block, block.pos)

  #se crea la matriz que contendra las casillas limpias y sucias
  def createCeroMatrix(self):
    for i in range(0,self.rows):
      zeros = []
      for j in range(0,self.columns):
        zeros.append(0)
      self.matrix.append(zeros)
  
  #se colocan los robots
  def placeRobots(self):
    for _ in range(0,self.robots):
      robot = Robot(self, (1, 1))
     # solucion del problema 
       # sacamos una tupla aleatoria del arreglo que contiene las posiciones de los stacks
      if (len(self.stacksPos) > 0):
         
        stackIndex = self.random.randint(0, len(self.stacksPos)-1)
        tupleOfStack = self.stacksPos[stackIndex]
        # asignamos el siguiente movimiento como el stack
        robot.allocatedStack = tupleOfStack
        del self.stacksPos[stackIndex]
      self.arrRobots.append(robot)
     # solucion del problema 
      self.grid.place_agent(robot, robot.pos)
      self.schedule.add(robot)

  #se colocan los robots
  def placeStacks(self):
    for _ in range(0,self.Stacks):
      randomX = self.random.randint(0, self.rows-1)
      randomY = self.random.randint(0, self.columns-1)
      while self.matrix[randomX][randomY] == 1:
        randomX = self.random.randint(0, self.rows-1)
        randomY = self.random.randint(0, self.columns-1)
      stack = Stack(self,(randomY,randomX))
      self.grid.place_agent(stack, stack.pos)
      self.matrix[randomX][randomY] = 1
      position = (randomX, randomY)
      self.stacksPos.append(stack.pos)
      self.arrStacks.append(stack)
      self.schedule.add(stack)

#Agent portrayal permite rea
def agent_portrayal(agent):
  if(agent.type == "robot"):
    return {"Shape": "robot.png", "Layer": 0}
  elif(agent.type == "Box"):
    return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "#495057", "Layer": 1}
  elif(agent.type == "CleanBlock"):
    return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "#ced4da", "Layer": 1}
  elif(agent.type == "Stack"):
    if(agent.cajas == 5):
      agent.full = True
      return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "#b81365", "Layer": 1}
    return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "#bfab25", "Layer": 1}


grid = CanvasGrid(agent_portrayal, 10, 10, 450, 450)


#server = ModularServer(Maze, [grid], "Maze", {})
#server.port = 8521
#server.launch() 