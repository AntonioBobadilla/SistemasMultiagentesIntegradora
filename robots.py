
#Realizado por:
#José Antonio Bobadilla García A01734433
#Ultima modificacion: 30/11/2021


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
  # Atributo el cual nos dice si el robot tiene alguna caja en ese momento
    self.empty = True
  # Atributo que nos servira en unity para colocar las cajas apiladas
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
      if (self.model.Box == 0 and self.allRobotsEmpty()): # se checa que ya no haya cajas y que los robots no tengan cajas.
        print("se han recogido todas las cajas en un total de", self.getTotalMovements(),"movimientos")
      if(self.movements < self.model.time and clean):
        self.movements += 1
        self.model.grid.move_agent(self, next_move)
    else: # si el robot ya tiene 5 cajas debe dejar las cajas en el stack
      
      # sacamos el path hacia su stack especificado
      path = self.getPathToClosestStack(self.allocatedStack)

      if(self.index < len(path)): # si entra sigue en camino hacia el stack
        next_move = path[self.index]
        self.index = 1
        self.model.grid.move_agent(self, next_move)
      else: # si entra ya llegó a su stack
        self.index = 0 # se reinicia el indice para poder usarlo despues en otros paths hacia stack
        self.inventory = 0 # se setea su numero de cajas a 0
        self.leaveBoxes() # deja las cajas en su stack
        self.leavingBoxes = True 
        self.assignNewStack() # se le asigna un nuevo stack
      clean = True # continuamos con los steps
 
    #funcion auxiliar que retorna el numero total de movimientos por todos los robots
  def getTotalMovements(self):
    movements = 0
    for robot in self.model.arrRobots:
      movements += robot.movements
    return movements

    #funcion auxiliar que retorna si todos los robots están vacios  
  def allRobotsEmpty(self):
    for robot in self.model.arrRobots:
        if not robot.empty:
          return False
    return True    

   #funcion auxiliar que asigna un nuevo stack al robot
  def assignNewStack(self):
    if (len(self.model.stacksPos) > 0):
         
      stackIndex = self.random.randint(0, len(self.model.stacksPos)-1)
      tupleOfStack = self.model.stacksPos[stackIndex]
      # asignamos el siguiente movimiento como la posicion del stack
      self.allocatedStack = tupleOfStack
      # borramos ese stack del arreglo para que no sea utilizado por otros robots
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

   #funcion auxiliar que saca la ruta hacia el un stack vacio
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

  #funcion auxiliar que crea un arreglo de stacks que aún no están llenos
  def createArrayOfNotFullStacks(self):
    index = 0
    for stack in self.model.arrStacks:
        if stack.full:
          del self.model.arrStacks[index]
        index += 1

  #funcion auxiliar que dej las cajas en los stacks que aún no están llenos
  def leaveMissingBoxes(self):
    cellmates = self.model.grid.get_cell_list_contents([self.pos])
    for block in cellmates:
      if(block.type == "Stack" ):
       if (block.cajas < 5): # si el stack tiene 5 cajas entra 
         if ((block.cajas + self.inventory) < 5 ): # si el numero de cajas del stack + numero de cajas del robot es  mejor que 5 entra
           block.cajas = block.cajas + self.inventory # el numero de cajas será el de la suma de los 2
           self.empty = True # se setea el robot como vacio
           self.inventory = 0  # se setea el número de cajas del robot a 0
         elif (((block.cajas + self.inventory) == 5 )): # si el numero de cajas del stack + numero de cajas del robot es igual a 5 entra
            block.cajas = 5 # se asigna el numero de cajas máximo al stack
            self.inventory = 0 # se setea su inventario a 0
            block.full = True # se setea el stack como lleno
         else: # si el numero de cajas del stack + numero de cajas del robot es mayor a 5 entra
           spareBoxes = (block.cajas + self.inventory) - 5 # se calculan las cajas sobrantes
           block.cajas = 5 # se asigna el numero de cajas máximo al stack
           block.full = True
           self.inventory = spareBoxes  # se asignan las cajas restantes al robot
           
# funcion auxiliar que se dirije a un stack vacio
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

  #funcion auxiliar que revisa si la casilla donde se encuentra el robot contiene una caja o no
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
      if(block.type == "Box" ): # si es una caja 
        block.limpio = True # se recoje
        block.stackAsignadoX = self.allocatedStack[0] # se añaden coordenadas de stack que tiene el robot a caja 
        block.stackAsignadoY = self.allocatedStack[1]
        block.changeColor() # cambia el color del stack, significando que está lleno
        self.limpiadas += 1 # se aumenta el número de cajas recogidas totales por el robot
        self.inventory += 1 # se aumenta el número de cajas recogidas por el robot
        block.inStack = self.inventory # el numero de caja que recogio el robot se le asigna a la caja para que unity coloque las cajas apiladas
        self.model.Box -= 1 # se resta una caja al modelo principal
        return False
      else:
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

#Agente Stack
class Stack(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    self.type = "Stack"
    self.cajas = 0
    self.full = False


class Maze(Model):
  #se asignan las variables modificables por el usuario siendo filas, columnas, robots, tiempo de ejecucion y el numero de cajas
  def __init__(self, rows =10,columns = 10, robots = 5, time = 20000, Box = 60):
    super().__init__()
    self.schedule = RandomActivation(self)
    self.rows = rows
    self.columns = columns
    self.robots = robots
    self.time = time
    self.Box = Box
    self.Stacks = (Box // 5)
    self.grid = MultiGrid(self.columns, self.rows, torus=False)
    self.matrix = []
    self.stacksPos = []
    self.arrStacks = []
    self.arrRobots = []
    #se crea una matriz de ceros para identificar las casillas con cajas y sin cajas
    self.createCeroMatrix()
    #se crean los robots y bloques tanto cajas para colocarse en el tablero
    self.placeStacks()
    self.placeRobots()
    self.placeBox()
    self.placeCleanBlocks()
 

  def step(self):
    self.schedule.step()
  
  @staticmethod
  def count_type(model):
      return model.Box

  #se crean las cajas de manera aleatoria
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

  #se crea la matriz que contendra las casillas limpias y con cajas
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
       # sacamos una tupla aleatoria del arreglo que contiene las posiciones de los stacks
      if (len(self.stacksPos) > 0):
         
        stackIndex = self.random.randint(0, len(self.stacksPos)-1)
        tupleOfStack = self.stacksPos[stackIndex]
        # asignamos el siguiente movimiento como el stack
        robot.allocatedStack = tupleOfStack
        del self.stacksPos[stackIndex]
      self.arrRobots.append(robot)
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