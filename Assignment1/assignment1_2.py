import turtle 

turtle.position()
turtle.pendown()
turtle.delay(200)

angle = 0

for i in range(0,3):
	
	turtle.forward(500)
	angle = angle+120
	turtle.setheading(angle)

	turtle.forward(250)
	angle = angle+120
	turtle.setheading(angle)

turtle.position()