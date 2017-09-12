import re


aim=re.compile(r'([A-Za-z]+\s*[A-Za-z]*)',re.S)
with open('result.txt','r') as f:
    result=re.findall(aim,f.read())
    print(result[0])

