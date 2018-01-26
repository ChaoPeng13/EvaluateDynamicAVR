\ENCODING=ISO-8859-1
\Problem name: IloCplex

Maximize
 obj: 576 mode_1 + 965 mode_0 + 424 mode_2 + 343 mode_3 + 277 mode_4
      + 246 mode_5 + x39
Subject To
 constraint_1:     mode_1 >= 1
 constraint_2_0:   mode_0 <= 5
 constraint_2_1:   mode_1 <= 9
 constraint_2_2:   mode_2 <= 5
 constraint_2_3:   mode_3 <= 0
 constraint_2_4:   mode_4 <= 0
 constraint_2_5:   mode_5 <= 0
 constraint_3_max: mode_1 + mode_0 + mode_2 + mode_3 + mode_4 + mode_5 <= 9
 constraint_4_1:   x7 + x9 + x11 + x13 >= 1
 constraint_4_2:   x15 + x17 + x19 + x21 >= 1
 constraint_4_3:   x23 + x25 + x27 + x29 >= 1
 constraint_4_4:   x31 + x33 + x35 + x37 >= 1
 constraint_5:     24000 mode_1 + 40000 mode_0 + 17142 mode_2 + 13333 mode_3
                   + 10909 mode_4 + 9230 mode_5 <= 239999
 i1:  x7 = 1 <-> mode_0 <= 0
 i2:  x9 = 1 <-> mode_2 <= 0
 i3:  x11 = 1 <-> mode_1 >= 3
 i4:  x13 = 1 <-> mode_1 >= 3
 i5:  x15 = 1 <-> mode_1 <= 0
 i6:  x17 = 1 <-> mode_3 <= 0
 i7:  x19 = 1 <-> mode_2 >= 5
 i8:  x21 = 1 <-> mode_2 >= 5
 i9:  x23 = 1 <-> mode_2 <= 0
 i10: x25 = 1 <-> mode_4 <= 0
 i11: x27 = 1 <-> mode_3 >= 6
 i12: x29 = 1 <-> mode_3 >= 6
 i13: x31 = 1 <-> mode_3 <= 0
 i14: x33 = 1 <-> mode_5 <= 0
 i15: x35 = 1 <-> mode_4 >= 8
 i16: x37 = 1 <-> mode_4 >= 8
Bounds
      mode_1 >= 0
      mode_0 >= 0
      mode_2 >= 0
      mode_3 >= 0
      mode_4 >= 0
      mode_5 >= 0
 0 <= x7 <= 1
 0 <= x9 <= 1
 0 <= x11 <= 1
 0 <= x13 <= 1
 0 <= x15 <= 1
 0 <= x17 <= 1
 0 <= x19 <= 1
 0 <= x21 <= 1
 0 <= x23 <= 1
 0 <= x25 <= 1
 0 <= x27 <= 1
 0 <= x29 <= 1
 0 <= x31 <= 1
 0 <= x33 <= 1
 0 <= x35 <= 1
 0 <= x37 <= 1
      x39 = 0
Binaries
 x7  x9  x11  x13  x15  x17  x19  x21  x23  x25  x27  x29  x31  x33  x35  x37 
Generals
 mode_1  mode_0  mode_2  mode_3  mode_4  mode_5 
End
