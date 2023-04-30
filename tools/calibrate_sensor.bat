set /P comportnum="COM Port: "
python tino2cal_v01.py COM%comportnum% 57600 -pwd
pause