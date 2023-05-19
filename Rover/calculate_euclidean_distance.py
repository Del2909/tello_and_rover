import math

distances =[[0.0, 0.0], [-1.212477684020996, -9.49442982673645], [-1.039421558380127, -9.017062187194824], [-0.7969677448272705, -8.565366268157959], [-0.5797326564788818, -8.227699995040894], [-0.40621161460876465, -7.996930181980133], [-0.27084946632385254, -7.847106456756592], [-0.1624584197998047, -7.755640149116516], [-0.06921887397766113, -7.707787677645683], [0.020325183868408203, -7.697014138102531], [-10.76311469078064, 0.058075785636901855], [-10.314220190048218, 0.01383274793624878], [-10.102222859859467, -0.008878111839294434], [-10.021884739398956, -0.014019757509231567], [-9.997694566845894, -0.00635981559753418]]


tot_x = 0
tot_y = 0

for distance in distances:
    x = distance[0]
    y = distance[1]
    tot_x += x
    tot_y += y


direct_distance = math.sqrt(tot_x**2 + tot_y **2)


print("FINAL_DISTANCE: ", direct_distance)




# DESIRED FORMAT: DISTANCE, ANGLE, CHARGING
test_values = [[0,0,1], [0,90, 0], [30, 0,0], [0,-90,0],[30, 0,0], [0,90,0], [30,0,0],[0,90,0],[30,0,0],[0,-90,0],[30,0,0], [0,0,1]]




test_2_values = [[30, 90], [30,-90], [30,90],[30,90],[30,-90]]


test_3_values = [[0,90], [[3, 0] for x in range(10)], [0,-90], [[3, 0] for x in range(10)], [0,90],  [[3, 0] for x in range(10)],[0,90], [[3, 0] for x in range(10)],[0,-90], [[3, 0] for x in range(10)]]
