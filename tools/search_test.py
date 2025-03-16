import time

data = [
    (	0.01	,	0.398107171	)	,
    (	0.02	,	0.501187234	)	,
    (	0.02	,	0.630957344	)	,
    (	0.03	,	0.794328235	)	,
    (	0.03	,	1	)	,
    (	0.04	,	1.258925412	)	,
    (	0.07	,	1.995262315	)	,
    (	0.11	,	3.16227766	)	,
    (	0.13	,	3.981071706	)	,
    (	0.34	,	10	)	,
    (	1.06	,	31.6227766	)	,
    (	2.12	,	63.09573445	)	,
    (	3.36	,	100	)	,
    (	3.76	,	112.2018454	)	,
    (	4.22	,	125.8925412	)	,
    (	5.32	,	158.4893192	)	,
    (	6.70	,	199.5262315	)	,
    (	8.46	,	251.1886432	)	,
    (	10.66	,	316.227766	)	
]

find_val = float(input("Point: "))

found = False
top = len(data)
bot = 0
a = None
b = None
while not found:
    print(f'Search range is between [{bot}:{top}) ...', end = ' ')
    i = int(((top - bot) / 2) + bot)
    print(f'i = {i}')
    if i <= 1:
        break
    
    a = data[i - 1]
    b = data[i]

    found = (find_val > a[0]) and (find_val <= b[0])

    if not found:
        if find_val <= a[0]:
            top = i
        elif find_val > b[0]:
            bot = i
    
    time.sleep(2.0)

print(f'Value {find_val:.4f} is between {a[0]:.4f} and {b[0]:.4f}')