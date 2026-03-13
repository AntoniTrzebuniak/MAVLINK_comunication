import math

h = 120
RESOLUTION1 = (256,192)
RESOLUTION2 = (640,512)

alpha = math.radians(48.3 / 2)  # połowa HFOV
beta = math.radians(38.6 / 2)   # połowa VFOV

width = 2 * h * math.tan(alpha)
height = 2 * h * math.tan(beta)

delta = math.radians(70)
distance = h*math.tan(delta)

print(f"distance: {distance}")
print(width, height)
print('metrów na pixel (H*V)')
print(width/RESOLUTION2[0], height/RESOLUTION2[1])
