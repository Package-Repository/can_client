##############################################

def GENERATE_RANDOM_NODE_IDENTIFIER():
    """ 
        Generate random number between 
        0 and 1 million and returns as string
    """
    import random
    NUM_RANDOM_NAMES = 1000000
    return str(int(random.random() * NUM_RANDOM_NAMES))

##############################################