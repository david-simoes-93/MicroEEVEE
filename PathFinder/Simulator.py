from enum import Enum


class Cell:
    def __init__(self, up, down, left, right):
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.goal = False

    def set(self, up, down, left, right):
        self.up = up
        self.down = down
        self.left = left
        self.right = right


MAP_SIZE = 20
CM_PER_CELL = 12.5


class MazeSimulator:
    def __init__(self):
        # linhas pretas de 2.5cm
        # área total de 240cmx240cm.
        # A distância mínima entre o eixo central de duas linhas paralelas é 12.5 cm
        # o espaço mínimo entre os centros de duas interseções ou entre o centro de uma interseção e uma mudança de direção é 25 cm
        # O ponto de chegada do labirinto é marcado com um retângulo preto com 12.5x5.5 cm.
        # Essa marca tem uma distância mínima à linha mais próxima de 5 cm e o segmento de reta que a ela conduz tem um comprimento mínimo de 20cm.

        self.maze = [[Cell(False, False, False, False)
                      for y in range(MAP_SIZE)] for x in range(MAP_SIZE)]
        self.starting_pos = [4, 0]

        # example map from https://drive.google.com/file/d/1_pOQQkb6gatRcMIIKV23zb5OahAf3XqW/view
        self.maze[0][0].set(up=False, down=True, left=False, right=True)
        self.maze[1][0].set(up=False, down=False, left=True, right=True)
        self.maze[2][0].set(up=False, down=False, left=True, right=True)
        self.maze[3][0].set(up=False, down=True, left=True, right=True)
        self.maze[4][0].set(up=False, down=False, left=True, right=True)
        self.maze[5][0].set(up=False, down=True, left=True, right=True)
        self.maze[6][0].set(up=False, down=False, left=True, right=True)
        self.maze[7][0].set(up=False, down=False, left=True, right=False)
        self.maze[8][0].set(up=False, down=True, left=False, right=False)

        self.maze[0][1].set(up=True, down=True,left=False, right=False)
        self.maze[1][1].set(up=False, down=False, left=False, right=False)
        self.maze[2][1].set(up=False, down=False, left=False, right=False)
        self.maze[3][1].set(up=True, down=True, left=False, right=False)
        self.maze[4][1].set(up=False, down=False, left=False, right=False)
        self.maze[5][1].set(up=True, down=True, left=False, right=False)
        self.maze[6][1].set(up=False, down=False, left=False, right=False)
        self.maze[7][1].set(up=False, down=False, left=False, right=False)
        self.maze[8][1].set(up=True, down=True, left=False, right=False)

        self.maze[0][2].set(up=True, down=False, left=False, right=True)
        self.maze[1][2].set(up=False, down=False, left=True, right=True)
        self.maze[2][2].set(up=False, down=True, left=True, right=False)
        self.maze[3][2].set(up=True, down=False, left=False, right=False)
        self.maze[4][2].set(up=False, down=False, left=False, right=False)
        self.maze[5][2].set(up=True, down=True, left=False, right=False)
        self.maze[6][2].set(up=False, down=False, left=False, right=True)
        self.maze[7][2].set(up=False, down=False, left=True, right=True)
        self.maze[8][2].set(up=True, down=True, left=True, right=False)

        self.maze[0][3].set(up=False, down=False, left=False, right=False)
        self.maze[1][3].set(up=False, down=False, left=False, right=False)
        self.maze[2][3].set(up=True, down=True, left=False, right=False)
        self.maze[3][3].set(up=False, down=False, left=False, right=True)
        self.maze[3][3].goal = True
        self.maze[4][3].set(up=False, down=False, left=True, right=True)
        self.maze[5][3].set(up=True, down=True, left=True, right=True)
        self.maze[6][3].set(up=False, down=False, left=True, right=True)
        self.maze[7][3].set(up=False, down=False, left=True, right=False)
        self.maze[8][3].set(up=True, down=True, left=False, right=False)

        self.maze[0][4].set(up=False, down=True, left=False, right=True)
        self.maze[1][4].set(up=False, down=False, left=True, right=True)
        self.maze[2][4].set(up=True, down=True, left=True, right=True)
        self.maze[3][4].set(up=False, down=False, left=True, right=True)
        self.maze[4][4].set(up=False, down=False, left=True, right=False)
        self.maze[5][4].set(up=True, down=True, left=False, right=False)
        self.maze[6][4].set(up=False, down=False, left=False, right=True)
        self.maze[7][4].set(up=False, down=False, left=True, right=True)
        self.maze[8][4].set(up=True, down=True, left=True, right=False)

        self.maze[0][4].set(up=False, down=True, left=False, right=True)
        self.maze[1][4].set(up=False, down=False, left=True, right=True)
        self.maze[2][4].set(up=True, down=True, left=True, right=True)
        self.maze[3][4].set(up=False, down=False, left=True, right=True)
        self.maze[4][4].set(up=False, down=False, left=True, right=False)
        self.maze[5][4].set(up=True, down=True, left=False, right=False)
        self.maze[6][4].set(up=False, down=False, left=False, right=True)
        self.maze[7][4].set(up=False, down=False, left=True, right=True)
        self.maze[8][4].set(up=True, down=True, left=True, right=False)

        self.maze[0][5].set(up=True, down=True, left=False, right=False)
        self.maze[1][5].set(up=False, down=False, left=False, right=True)
        self.maze[2][5].set(up=False, down=False, left=True, right=True)
        self.maze[3][5].set(up=False, down=True, left=True, right=True)
        self.maze[4][5].set(up=False, down=False, left=True, right=True)
        self.maze[5][5].set(up=True, down=True, left=True, right=True)
        self.maze[6][5].set(up=False, down=False, left=True, right=True)
        self.maze[7][5].set(up=False, down=False, left=True, right=False)
        self.maze[8][5].set(up=True, down=True, left=False, right=False)

        self.maze[0][6].set(up=True, down=True, left=False, right=True)
        self.maze[1][6].set(up=False, down=False, left=True, right=True)
        self.maze[2][6].set(up=False, down=True, left=True, right=False)
        self.maze[3][6].set(up=True, down=True, left=False, right=False)
        self.maze[4][6].set(up=False, down=True, left=False, right=False)
        self.maze[5][6].set(up=True, down=True, left=False, right=False)
        self.maze[6][6].set(up=False, down=False, left=False, right=False)
        self.maze[7][6].set(up=False, down=False, left=False, right=False)
        self.maze[8][6].set(up=True, down=True, left=False, right=False)

        self.maze[0][7].set(up=True, down=True, left=False, right=False)
        self.maze[1][7].set(up=False, down=False, left=False, right=False)
        self.maze[2][7].set(up=True, down=True, left=False, right=False)
        self.maze[3][7].set(up=True, down=False, left=False, right=False)
        self.maze[4][7].set(up=True, down=True, left=False, right=False)
        self.maze[5][7].set(up=True, down=False, left=False, right=True)
        self.maze[6][7].set(up=False, down=False, left=True, right=True)
        self.maze[7][7].set(up=False, down=False, left=True, right=True)
        self.maze[8][7].set(up=True, down=False, left=True, right=False)

        self.maze[0][8].set(up=True, down=False, left=False, right=False)
        self.maze[1][8].set(up=False, down=False, left=False, right=False)
        self.maze[2][8].set(up=True, down=False, left=False, right=True)
        self.maze[3][8].set(up=False, down=False, left=True, right=True)
        self.maze[4][8].set(up=True, down=False, left=True, right=True)
        self.maze[5][8].set(up=False, down=False, left=True, right=True)
        self.maze[6][8].set(up=False, down=False, left=True, right=False)
        self.maze[7][8].set(up=False, down=False, left=False, right=False)
        self.maze[8][8].set(up=False, down=False, left=False, right=False)


