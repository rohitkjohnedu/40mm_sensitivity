# Aim
To simulate how the following affects how close the experiment and simulation matches
- Young
- Density
- radius


# Instructions
Copy the contents of the folder libs to 
```
$ ~/.local/lib/python3.8/site-packages/
```

- or the appropriate python version


For testing the code use 
```
$ yade-batch -j=4 --log ./logs/$.%.log test_parametric.table main.py
```

For full simulation use
```
$ yade-batch -j=4 --log ./logs/$.%.log og_parametric.table main.py
```

- -j=4 means 4 cores/thread. Change to an appropriate number 
