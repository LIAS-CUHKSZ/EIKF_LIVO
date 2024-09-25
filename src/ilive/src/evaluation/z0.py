import pandas as pd

def z0(file,out):
   
    data = pd.read_csv(file, dtype=float)
    data.iloc[:,5] = data.iloc[:,5]-data.iloc[:,5];   
    data.to_csv(out,index=False)
