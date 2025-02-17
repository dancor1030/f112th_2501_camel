import matplotlib.pyplot as plt

# Data
data = [80, 20]  # Values for each slice
labels = ['Apples', 'Bananas']  # Labels for each slice
colors = ['blue', 'red']  # Colors for each slice

# Set the angle for a specific category
highlight_category = 'Bananas'  # The category you want to set at a specific angle
highlight_index = labels.index(highlight_category)  # Index of the category

# Calculate the start angle so that the highlighted category starts at the desired angle
start_angle = 60  # Angle from the horizontal (in degrees)
if highlight_index > 0:
    # Adjust the start angle based on the cumulative sum of previous slices
    start_angle -= sum(data[:highlight_index]) / sum(data) * 360

# Create the pie chart
plt.pie(data, labels=labels, colors=colors, startangle=start_angle, autopct='%1.1f%%')

# Set aspect ratio to be equal so the pie chart is circular
plt.axis('equal')

# Add a title
plt.title('Pie Chart with Highlighted Category at an Angle')

# Show the plot
plt.show()