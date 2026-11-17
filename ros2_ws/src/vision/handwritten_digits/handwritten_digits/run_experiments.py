#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# PRACTICE 03 - EXPERIMENTS RUNNER
#
import time
import csv
import os
import sys
import io
import contextlib
import numpy
from fc import FCNeuralNetwork


def load_dataset(folder, encoding="one_hot"):
    if encoding == "one_hot":
        labels_def = [[1,0,0,0,0,0,0,0,0,0], [0,1,0,0,0,0,0,0,0,0], [0,0,1,0,0,0,0,0,0,0],
                      [0,0,0,1,0,0,0,0,0,0], [0,0,0,0,1,0,0,0,0,0], [0,0,0,0,0,1,0,0,0,0],
                      [0,0,0,0,0,0,1,0,0,0], [0,0,0,0,0,0,0,1,0,0], [0,0,0,0,0,0,0,0,1,0],
                      [0,0,0,0,0,0,0,0,0,1]]
        out_dim = 10
    else:
        labels_def = [[0,0,0,0], [0,0,0,1], [0,0,1,0], [0,0,1,1], [0,1,0,0],
                      [0,1,0,1], [0,1,1,0], [0,1,1,1], [1,0,0,0], [1,0,0,1]]
        out_dim = 4

    training_x, training_t, testing_x, testing_t = [], [], [], []
    for i in range(10):
        f_path = os.path.join(folder, "data" + str(i))
        f_data = [c/255.0 for c in open(f_path, "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784, 1]) for j in range(1000)]
        label = numpy.asarray(labels_def[i]).reshape([out_dim, 1])
        training_x += images[:500]
        training_t += [label] * 500
        testing_x  += images[500:]
        testing_t  += [label] * 500
    return training_x, training_t, testing_x, testing_t


def is_correct(y, label, encoding):
    if encoding == "one_hot":
        # Same criterion as fc.py: euclidean distance < 0.5
        return numpy.linalg.norm(label - y) < 0.5
    else:
        # Binary: round each bit and compare exactly
        y_bits = (y > 0.5).astype(int)
        label_bits = label.astype(int)
        return numpy.array_equal(y_bits, label_bits)


def run_experiment(arch, epochs, batch_size, lr, train_x, train_t, test_x, test_t, encoding):
    nn = FCNeuralNetwork(arch)
    t0 = time.time()
    # Silence training prints to keep the log readable
    with contextlib.redirect_stdout(io.StringIO()):
        nn.train_by_SGD(train_x, train_t, epochs, batch_size, lr)
    elapsed = time.time() - t0

    correct = 0
    for img, label in zip(test_x, test_t):
        y = nn.feedforward(img)[-1]
        if is_correct(y, label, encoding):
            correct += 1
    accuracy = correct / len(test_x) * 100.0
    return elapsed, accuracy


def get_experiments(encoding):
    out_dim = 10 if encoding == "one_hot" else 4
    arch_a = [784, 30, out_dim]
    arch_b = [784, 50, 30, out_dim]

    experiments = []
    # Vary learning rate (epochs=10, batch=30, arch_a)
    for lr in [0.5, 1.0, 3.0, 10.0]:
        experiments.append(("vary_lr", arch_a, 10, 30, lr))
    # Vary epochs (lr=3.0, batch=30, arch_a)
    for ep in [3, 10, 50, 100]:
        experiments.append(("vary_epochs", arch_a, ep, 30, 3.0))
    # Vary batch size (lr=3.0, epochs=10, arch_a)
    for bs in [5, 10, 30, 100]:
        experiments.append(("vary_batch", arch_a, 10, bs, 3.0))
    # Alternative architecture (vary lr to allow comparison)
    for lr in [0.5, 1.0, 3.0, 10.0]:
        experiments.append(("vary_lr_arch_b", arch_b, 10, 30, lr))

    return experiments


def main():
    dataset_folder = "../dataset"
    output_csv = "resultados_experimentos.csv"

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["encoding", "study", "architecture", "epochs",
                         "batch_size", "learning_rate", "train_time_s",
                         "accuracy_percent"])

    total_t0 = time.time()

    for encoding in ["one_hot", "binary"]:
        print("\n" + "=" * 70)
        print(f"  EXPERIMENTS: {encoding.upper()}")
        print("=" * 70)
        print(f"\nLoading dataset ({encoding}) ...")
        train_x, train_t, test_x, test_t = load_dataset(dataset_folder, encoding)
        print(f"  Training: {len(train_x)} examples | Testing: {len(test_x)} examples\n")

        experiments = get_experiments(encoding)
        n = len(experiments)

        for i, (study, arch, epochs, batch_size, lr) in enumerate(experiments, 1):
            print(f"[{i}/{n}] {study} | arch={arch} | epochs={epochs} | "
                  f"batch={batch_size} | lr={lr}")
            try:
                elapsed, accuracy = run_experiment(arch, epochs, batch_size, lr,
                                                    train_x, train_t, test_x, test_t,
                                                    encoding)
                print(f"        -> time={elapsed:.2f}s | accuracy={accuracy:.2f}%")
                with open(output_csv, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([encoding, study, str(arch), epochs, batch_size,
                                     lr, f"{elapsed:.4f}", f"{accuracy:.2f}"])
            except Exception as e:
                print(f"        -> ERROR: {e}")
                with open(output_csv, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([encoding, study, str(arch), epochs, batch_size,
                                     lr, "ERROR", "ERROR"])

    total_elapsed = time.time() - total_t0
    print("\n" + "=" * 70)
    print(f"  DONE. Total time: {total_elapsed/60:.1f} min")
    print(f"  Results saved to: {output_csv}")
    print("=" * 70)


if __name__ == "__main__":
    main()
