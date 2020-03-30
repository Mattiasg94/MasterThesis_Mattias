from setup import obs_move_line

lane=1
x=20
y=10
v=1
x,y=obs_move_line(lane, v, x, y)

print(x,y)