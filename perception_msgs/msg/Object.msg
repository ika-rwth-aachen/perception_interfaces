# Message for the state of an object

# Lifetime ID of the object
uint64 id

# Existence probability of the object [0,1]
float64 existence_probability

# Motion state estimate of the object
ObjectState state

# History of the object's states
ObjectState[] state_history

# Prediction of the object state
ObjectStatePrediction[] state_predictions