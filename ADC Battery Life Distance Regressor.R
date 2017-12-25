# Battery Life Predictor
# Predicts how far a drone can travel given certain weather conditions.

# Importing the dataset
dataset <- read.csv('Drone Battery Life Data.csv')

# Splitting the dataset into a Training and Testing set
library(caTools)
split <- sample.split(dataset$Distance.Traveled, SplitRatio = 0.9)
training_set <- subset(dataset, split == T)
testing_set <- subset(dataset, split == F)

# Fitting Regression to the Data set
regressor <- lm(Distance.Traveled..miles. ~ .,
                data = training_set)