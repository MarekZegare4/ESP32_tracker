import math

# http://walter.bislins.ch/bloge/index.asp?page=Derivation+of+Angle+to+Target+Top+Equation
def angleToTarget(observerZ, targetZ, distance):
    R = 6374.3
    a = R + observerZ
    b = R + targetZ
    gamma = distance / R
    alpha = -math.asin((a - b * math.cos(gamma))/(math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(gamma))))
    return math.degrees(alpha)

print(angleToTarget(111444/1000, 72036/1000, 24))