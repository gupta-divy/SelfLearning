
class Node():
    def __init__(self,x,y) -> None:
        self.x=x
        self.y=y
        self.g=0
        self.h=0
        self.f=self.est_f()
    
    def est_f(self) -> float:
        return  self.h+self.g    

class Env():
    def __init__(self, rows:int = 10, cols:int = 10, blocked:list = None):
        self.rows = rows
        self.cols = cols
        self.grid = [[Node(x,y) for x in range(cols)] for y in range(rows)]
        self.blocked = blocked
    
    def get_neighbors(self, x, y):
        neighbors=[]
        if(x>0):neighbors.append((x-1,y))
        if(x<self.cols-1):neighbors.append((x+1,y))
        if(y>0):neighbors.append((x,y-1))
        if(y<self.cols-1):neighbors.append((x,y+1))
        return neighbors

def h_cost(curr:tuple,goal:tuple)->float:
    'Manhattan Distance'
    return abs(curr[0]-goal[0])+abs(curr[1]-goal[1])

def a_star(start,end,env:Env):
    open_list = []
    closed_list = []
    open_list.push(start)
    while len(open_list)!=0:
        



