import math

print("const float sin_table[360] = {")
for i in range(360):
    val = math.sin(math.radians(i))
    # format as float with f suffix
    print(f"    {val:.8f}f,", end="")
    if (i + 1) % 5 == 0:
        print()
print("};")
