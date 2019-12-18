#linedetector
def trace(self):
    if self.line_detector.parkingMatch() == True:
    self.parking()

#autodrive
def parking(self):
    parking_0, parking_3, parking_4, parking_6 = self.obstacle_detector.for_parking_distance()
    while parking_6 <= 50:
    for go1 in range(5):
        drive(85, 110)
        if parking_6 >= 50:
            time.sleep(1)
            break
        time.sleep(0.1)

    for stop1 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(65, 110)
        time.sleep(0.1)

    for left1 in range(50):
        drive(65, 110)
        if data_0 <= 5:
            time.sleep(1)
            break
        time.sleep(0.1)

    for stop2 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(180, 70)
        time.sleep(0.1)

    for back1 in range(30):
        drive(180, 70)
        if data_3 <= 5:
            time.sleep(1)
            break
        time.sleep(0.1)

    for stop3 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(70, 110)
        time.sleep(0.1)

    for go2 in range(20):
        drive(70, 110)
        if data_0 <= 5:
            time.sleep(1)
            break
        time.sleep(0.1)

    for stop4 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(140, 70)
        time.sleep(0.1)

    for back2 in range(5):
        drive(140, 70)
        time.sleep(0.1)

    for stop5 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(90, 110)
        time.sleep(0.1)

    for go3 in range(5):
        drive(90, 110)
        time.sleep(0.1)

    for stop6 in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(90, 70)
        time.sleep(0.1)

    for back3 in range(50):
        drive(90, 70)
        if data_4 <= 5:
            time.sleep(1)
            break
        time.sleep(0.1)
    for finish in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(90, 90)
        time.sleep(0.1)
    rate.sleep()
    break
rospy.on_shutdown(exit_node)


#Obstacle
def for_parking(self,data):
	self.parking_0 = self.filter(self.filter_0,data.data[0],self.weights)
	self.parking_3 = self.filter(self.filter_3,data.data[3],self.weights)
	self.parking_4 = self.filter(self.filter_4,data.data[4],self.weights)
	self.parking_6 = self.filter(self.filter_6,data.data[6],self.weights)


def for_parking_distance(self):
	return self.parking_0 , self.parking_3 , self.parking_4, self.parking_6


