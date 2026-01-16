import random
import math

# We train a tiny logistic regression by hand (no external libraries)
# Model: p = sigmoid(w0 + w1*(1/(d+eps)) + w2*(closing_speed))
# Output weights to python_ai/weights.txt

def sigmoid(z):
    return 1 / (1 + math.exp(-z))

def generate_sample():
    # sonar distance d (meters)
    d = random.uniform(0.2, 30.0)

    # closing speed (m/s): positive means obstacle getting closer
    closing_speed = random.uniform(-2.0, 2.0)

    # "true risk" rule (synthetic labels):
    # risk high when distance small and closing speed positive
    risk_score = (1.2 / (d + 0.1)) + (0.8 * max(0.0, closing_speed))

    y = 1 if risk_score > 0.9 else 0  # obstacle danger label
    return d, closing_speed, y

def train():
    random.seed(42)

    # weights
    w0, w1, w2 = -2.0, 6.0, 1.5
    lr = 0.15

    eps = 0.05
    epochs = 3000

    for _ in range(epochs):
        d, cs, y = generate_sample()

        x1 = 1.0 / (d + eps)
        x2 = cs * 0.05  # scale closing speed feature

        z = w0 + w1 * x1 + w2 * x2
        p = sigmoid(z)

        # gradients for logistic regression
        # loss derivative: (p - y)
        grad = (p - y)
        w0 -= lr * grad
        w1 -= lr * grad * x1
        w2 -= lr * grad * x2

    return w0, w1, w2

if __name__ == "__main__":
    w0, w1, w2 = train()
    print("Trained weights:")
    print("w0 =", w0)
    print("w1 =", w1)
    print("w2 =", w2)

    with open("weights.txt", "w") as f:
        f.write(f"{w0} {w1} {w2}\n")

    print("\n✅ Saved to python_ai/weights.txt")
