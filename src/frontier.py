import heapq


class Frontier():
    def __init__(self):     
        self.data = []
        heapq.heapify(self.data)
   
    
    def __bool__(self):
        return len(self.data) > 0
    

    def push(self, elem):
        heapq.heappush(self.data, elem)


    def pop(self):
        return heapq.heappop(self.data)
    
    
    def find(self, node):
        for i in range(len(self.data)):
            cost, path = self.data[i]
            
            if path[-1] == node:
                return True, cost, path
    
        return False, None, None