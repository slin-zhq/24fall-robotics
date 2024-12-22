#code reference

先把arm endaffector初始設在(50,50,z)  z=100
預設camera 永遠朝下, 這樣才可以把depth 寫死(之後要改可以變成用depth camera 的資料)
camera coordinate 如上圖

#[[相機參數]]
# Camera parameters (need to find the one for the tm arm depth camera)
horizontal_fov =  1.5009832 # in radians 1.089
image_width = 640  # in pixels
image_height = 480  # in pixels

# Calculate the focal length
self.fx = image_width / (2 * np.tan(horizontal_fov / 2))
self.fy = self.fx  # Assuming square pixels, so fx == fy

# Assume cx and cy are at the center of the image
self.cx = image_width / 2
self.cy = image_height / 2

z= 100+a  the depth of the block surface(for simplicity 我們固定一開始先把robot arm 移到某定高度(100), ORIENTATION朝下)
#(a: 理論上為end effector ~ arm camera 的z距離)
#下面現在回傳得到的座標為參考camera frame的座標, 我們之後要轉成參考base坐標系
u,v=the point got from HW3
x,y,z=project_pixel_to_point(self, u, v, z)
float[] var_pBlock_CamFrame={x,y,z,0,0,0}


#2D投影到3D
def project_pixel_to_point(self, u, v, z):
    if z <= 0:
        return None  # Depth must be positive

    # Reconstruct the 3D point from the 2D pixel coordinates and the depth
    x = (u - self.cx) * z / self.fx
    y = (v - self.cy) * z / self.fy

    return x, y, z
  
#3D投影到2D
def project_point_to_pixel(self, x, y, z):#project_point_to_pixel(self, x, y, z):
    if z == 0:
        return None  # Avoid division by zero

    # Project the 3D point onto the 2D image plane
    u = self.fx * (x / z) + self.cx
    v = self.fy * (y / z) + self.cy

    return int(u), int(v)

#=======================================================================================
a=量camera到endeffector x距
b=量camera到endeffector y距 
c=量camera到endeffector z距
x1=x-a
y1=y-b
z1=z-b
float[] var_pBlock_endeFrame={x1,y1,z1,0,0,0}
#=======================================================================================
#1. 【用 trans()】得到base到endeffector的轉換矩陣fu, 現在要從endeffector走到base
#float[] var_P1 = {100, -200, 300, 10, 20, 60}
#float[] var_P2 = {-400, 200, 50, -20, 30, -45}
#float[] var_trans_RB = trans(var_P1, var_P2)
#// {-500, 400, -250, -24.61587, -15.56518, -88.61369}
#float[] var_trans_i = trans(var_P1, var_P2, true)

float[] var_P1 = #我們最初設給robot的初始點位置 #{-400, 200, 50, -20, 30, -45}
float[] var_P2 = {0, -0, 0, 0, 0, 0} #base的pose
float[] var_trans_RB = trans(var_P1, var_P2) #從base到endeffector初始點的位移跟旋轉角度
// {-500, 400, -250, -24.61587, -15.56518, -88.61369}

#2. 【用 3.23 applytrans()】p154
float[] var_pBlock_baseFrame = applytrans(var_pBlock_endeFrame, var_trans_RB)
// {-400, 200, 50, -20, 30, -45.00001}

#上面是打三個點統一stack轉好(在while loop1中跑直到empty)
#3. apply to robot
3.1 讓robot endeffector 移到這: var_pBlock_baseFrame
3.2 移完夾
3.3 上移到var_pBlock_baseFrame(z+=5)的地方放掉
3.4 再跑下個loop2夾另一個block
#=========================================================================================
#3. apply to robot
3.1 讓robot endeffector 移到這: var_pBlock_baseFrame
3.2 移完夾
3.3 上移到var_pBlock_baseFrame(x=50, y=50, z+=5*loopCount)的地方放掉#xy自訂的要疊的地方
		z+5*loopCount:
		5: block高, 每多疊一個就要預設在drop到高一點的地方
		loopCount: 每夾完一個block (loop) loopCount+=1
3.4 再跑下個loop2夾另一個block
#===========================================================================================
