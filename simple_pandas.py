# first import the libraries 
import pandas as pd 
   
# Create a dataFrame using dictionary 
df=pd.DataFrame({"Name":['Tom','Nick','John','Peter'], 
                 "Age":[15,26,17,28]}) 
  
# Creates a dataFrame with 
# 2 columns and 4 rows 
print(df)