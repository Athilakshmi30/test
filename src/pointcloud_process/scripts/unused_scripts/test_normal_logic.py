f = open("org_grey_pannel_unit.txt",'r')
data_ponts=f.readlines()
#print("data_points",data_ponts)
new_fitted_points = []
for i in data_ponts:
     n = i.split("\n")[0]
     numbers = map(float, n.split(','))
     #print("numbers",numbers)
     new_fitted_points.append(numbers)
#print("new_fitted_points",new_fitted_points)
#exit()
diff_f_s_xz = -500
diff_f_t_xz = -500
diff_f_fo_xz = -500
diff_f_fi_xz = -500
diff_s_t_xz = -500
diff_s_fo_xz = -500
diff_s_fi_xz = -500
prev_point_exist = False

for ind, point in enumerate(new_fitted_points):
     #point.split(",")
     #out = map(int, point)
     #print("out",point.split(","))      
     first_point = new_fitted_points[ind]
     print("first_point",first_point[5])


     if len(new_fitted_points)-ind>1:
         second_point = new_fitted_points[ind+1]
         diff_f_s_xz = abs(float(first_point[5])-float(second_point[5]))
         print("second_point",second_point[5])

     if len(new_fitted_points)-ind>2:
         third_point = new_fitted_points[ind+2]
         diff_f_t_xz = abs(float(first_point[5])-float(third_point[5]))
         diff_s_t_xz = abs(float(second_point[5])-float(third_point[5]))
         print("third_point",third_point[5])

     if len(new_fitted_points)-ind>3:
         fourth_point = new_fitted_points[ind+3]
         diff_f_fo_xz = abs(float(first_point[5])-float(fourth_point[5]))
         diff_s_fo_xz = abs(float(second_point[5])-float(fourth_point[5]))
         print("fourth_point",fourth_point[5])


     #if len(new_fitted_points)-ind>4:
     #    fifth_point = new_fitted_points[ind+4]
     ##    diff_f_fi_xz = abs(float(first_point[5])-float(fifth_point[5]))
      #  diff_s_fi_xz = abs(float(second_point[5])-float(fifth_point[5]))
      #  print("fifth_point",fifth_point[5])
     #exit()

     change_threshold = 0.01
     equal_threshold = 0.005#0.005

     print("diff_f_s_xz",diff_f_s_xz)
     if(ind==len(new_fitted_points)-1):
         prev_point = new_fitted_points[ind-1]
         first_point[3]= prev_point[3]
         first_point[4]= prev_point[4]
         first_point[5]= prev_point[5]                
     elif diff_f_s_xz>change_threshold:
                    print("diff_s_t_xz",diff_s_t_xz)
                    print("diff_s_fo_xz",diff_s_fo_xz)
                    print("diff_s_fi_xz",diff_s_fi_xz)


 
                    
                    if (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz<equal_threshold) and (diff_s_fi_xz<equal_threshold ) :    # modify current point / first point by giving second point value
                         if not prev_point_exist:
                             print("Modify current point")
                             first_point[3] = second_point[3]
                             first_point[4] = second_point[4]
                             first_point[5] = second_point[5]
                    elif (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz<equal_threshold) and (diff_s_fi_xz>equal_threshold):  # modify second, third and fourth point
                         print("Modify 2 3 4 point")
                         # Modify second point
                         second_point[3] = first_point[3]
                         second_point[4] = first_point[4]
                         second_point[5] = first_point[5]

                         # Modify third point
                         third_point[3] = first_point[3]
                         third_point[4] = first_point[4]
                         third_point[5] = first_point[5]

                         # Modify fourth point
                         fourth_point[3] = first_point[3]
                         fourth_point[4] = first_point[4]
                         fourth_point[5] = first_point[5]

                    elif (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz>equal_threshold) and (diff_s_fi_xz>equal_threshold or diff_s_fi_xz==-500):  # modify second, third 
                         print("Modify 2 3 point")
                         # Modify second point
                         second_point[3] = first_point[3]
                         second_point[4] = first_point[4]
                         second_point[5] = first_point[5]

                         # Modify third point
                         third_point[3] = first_point[3]
                         third_point[4] = first_point[4]
                         third_point[5] = first_point[5]


                    elif (diff_s_t_xz>equal_threshold or diff_s_t_xz==-500):  # modify second 
                         print("Modify 2 point")
                         # Modify second point
                         second_point[3] = first_point[3]
                         second_point[4] = first_point[4]
                         second_point[5] = first_point[5]


     first_point=[]
     second_point=[]
     third_point=[]
     fourth_point=[]
     fifth_point=[]
     diff_f_s_xz = -500
     diff_f_t_xz = -500
     diff_f_fo_xz = -500
     diff_f_fi_xz = -500
     diff_s_t_xz = -500
     diff_s_fo_xz = -500
     diff_s_fi_xz = -500
     prev_point_exist = True
     #exit()
print("modified fitted points",new_fitted_points)
