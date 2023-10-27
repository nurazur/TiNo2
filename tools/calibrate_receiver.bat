set /P comportnum="COM Port: "
python tino2cal_v02.py COM%comportnum% 230400 -pwd
pause