import os

for X in range(2,10):
    for Y in range(2,10):
        names = 'Goal' + str(X) + '_' + str(Y)
        newpath = r'ProcessedMoves2/' + names
        if not os.path.exists(newpath):
            os.makedirs(newpath)
