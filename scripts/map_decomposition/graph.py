class Graph:
    def __init__(self):
        self.vertices = {}
    
    def add_vertex(self, vertex):
        self.vertices[vertex] = []

    def add_edge(self, vertex_from, vertex_to):
        if vertex_to not in self.vertices[vertex_from]:
            self.vertices[vertex_from].append(vertex_to)
        
        if vertex_from not in self.vertices[vertex_to]:
            self.vertices[vertex_to].append(vertex_from)
        
    def adjacencyMatrix(self):
        if len(self.vertices) >= 1:
            self.vertex_names = sorted(self.vertices.keys())
            self.vertex_indices = dict(zip(self.vertex_names, range(len(self.vertex_names)))) 

            import numpy as np

            self.adjacency_matrix = np.zeros(shape=(len(self.vertices), len(self.vertices)))

            for i in range(len(self.vertex_names)):
                for j in range(i, len(self.vertices)):
                    for el in self.vertices[self.vertex_names[i]]:
                        j = self.vertex_indices[el]
                        self.adjacency_matrix[i,j] = 1
                        
            return self.adjacency_matrix
        else:
            return dict()

    def print(self):
        print(self.vertices)
        print()
        print(str(self.adjacencyMatrix()))


def main():
    g = Graph()

    a = (1, 2)
    b = (52, 3)
    c = (1, 1)
    d = (1, 3)
    e = (2, 2)
            
    g.add_vertex(a)
    g.add_vertex(b)
    g.add_vertex(c)
    g.add_vertex(d)
    g.add_vertex(e)

    g.add_edge(a, c)
    g.add_edge(a, d)
    g.add_edge(a, e)

    g.print()


if __name__ == "__main__":
    main()
