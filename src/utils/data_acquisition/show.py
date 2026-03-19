# ------------------------------------------------------------
# Imports
# ------------------------------------------------------------
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import mannwhitneyu

# ------------------------------------------------------------
# Constants
# ------------------------------------------------------------
FILE_WITH_DETECTION = "tour_times_with_detection.csv"
FILE_WITHOUT_DETECTION = "tour_times_without_detection.csv"

# ------------------------------------------------------------
# Data Loading and Preprocessing
# ------------------------------------------------------------
def load_data(file_path):
    """Load CSV, preprocess, and extract metrics."""
    data = pd.read_csv(file_path, skip_blank_lines=True)
    data = data.dropna(how="all")

    # Extract TOTAL rows
    total_rows = data[data["poi_name"] == "TOTAL"].copy()
    total_rows["time_seconds"] = total_rows["time_mmss"].apply(lambda x: int(x.split(":")[0]) * 60 + int(x.split(":")[1]))

    # Calculate recoveries and aborts
    total_recoveries = data["recoveries"].sum()
    total_aborts = data["aborts"].sum()
    num_tours = len(total_rows)
    average_recoveries = total_recoveries / num_tours if num_tours > 0 else 0
    average_aborts = total_aborts / num_tours if num_tours > 0 else 0

    # Filter out invalid rows
    data = data[~data["poi_name"].isin(["TOTAL", "############## END ##############"])]
    data = data.dropna(subset=["time_mmss"])
    data["time_seconds"] = data["time_mmss"].apply(lambda x: int(x.split(":")[0]) * 60 + int(x.split(":")[1]))

    return data, total_rows, total_recoveries, total_aborts, average_recoveries, average_aborts

def convert_in_seconds(data):
    data["time_seconds"] = data["time_mmss"].apply(lambda x: int(x.split(":")[0]) * 60 + int(x.split(":")[1]))
    return data

# ------------------------------------------------------------
# Metrics Calculation
# ------------------------------------------------------------
def calculate_average_total_time(total_rows):
    return total_rows["time_seconds"].mean()

# ------------------------------------------------------------
# Plotting Functions
# ------------------------------------------------------------
def plot_bar_comparison(x, values1, values2, labels, title, xlabel, ylabel, xticks, filename):
    plt.figure(figsize=(10, 6))
    plt.bar([i - 0.2 for i in x], values1, width=0.4, label=labels[0], alpha=0.7)
    plt.bar([i + 0.2 for i in x], values2, width=0.4, label=labels[1], alpha=0.7)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.xticks(x, xticks, rotation=45)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

def plot_single_bar(values, labels, title, ylabel, filename):
    plt.figure(figsize=(6, 4))
    plt.bar(labels, values, color=["blue", "orange"], alpha=0.7)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

def plot_boxplot(data, labels, title, ylabel, color, filename):
    plt.figure(figsize=(10, 6))
    plt.boxplot(data, labels=labels, patch_artist=True, boxprops=dict(facecolor=color), medianprops=dict(color="red"))
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

def plot_violinplot(data, labels, title, ylabel, filename):
    plt.figure(figsize=(10, 6))
    plt.violinplot(data, showmeans=True, showmedians=True)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xticks([1, 2], labels)
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

# ------------------------------------------------------------
# Main Execution
# ------------------------------------------------------------
def main():
    
    # Load data
    data_with_detection, total_with_detection, total_recoveries_with_detection, total_aborts_with_detection, average_recoveries_with_detection, average_aborts_with_detection = load_data(FILE_WITH_DETECTION)
    data_without_detection, total_without_detection, total_recoveries_without_detection, total_aborts_without_detection, average_recoveries_without_detection, average_aborts_without_detection = load_data(FILE_WITHOUT_DETECTION)

    # Convert TOTAL rows to seconds
    total_with_detection = convert_in_seconds(total_with_detection)
    total_without_detection = convert_in_seconds(total_without_detection)

    # Statistical test
    print("Mann-Whitney U test (total time):")
    print(mannwhitneyu(total_with_detection["time_seconds"], total_without_detection["time_seconds"], alternative='less'))

    # Group by POI
    grouped_with_detection = data_with_detection.groupby("poi_number").mean(numeric_only=True)
    grouped_without_detection = data_without_detection.groupby("poi_number").mean(numeric_only=True)
    # Trasforma l'indice in una lista di interi e crea etichette custom
    poi_numbers = list(map(int, grouped_with_detection.index))
    x = range(len(poi_numbers))
    poi_labels = [f"POI {num -1} to {num}" for num in poi_numbers]

    # Bar plots
    plot_bar_comparison(
        x,
        grouped_with_detection["recoveries"],
        grouped_without_detection["recoveries"],
        ["With Detection", "Without Detection"],
        "Recoveries Comparison",
        "POI",
        "Average Recoveries",
        poi_labels,
        "figure/recoveries_comparison.png"
    )
    plot_bar_comparison(
        x,
        grouped_with_detection["aborts"],
        grouped_without_detection["aborts"],
        ["With Detection", "Without Detection"],
        "Aborts Comparison",
        "POI",
        "Average Aborts",
        poi_labels,
        "figure/aborts_comparison.png"
    )
    plot_bar_comparison(
        x,
        grouped_with_detection["time_seconds"],
        grouped_without_detection["time_seconds"],
        ["With Detection", "Without Detection"],
        "Time per POI Comparison",
        "POI",
        "Average Time (seconds)",
        poi_labels,
        "figure/time_per_poi_comparison.png"
    )

    # Average total time
    avg_total_time_with = calculate_average_total_time(total_with_detection)
    avg_total_time_without = calculate_average_total_time(total_without_detection)
    plt.figure(figsize=(6, 4))
    plt.bar(0.3, [avg_total_time_with], width=0.1, label="With Detection", alpha=0.7, align='center')
    plt.bar([0.4], [avg_total_time_without], width=0.1, label="Without Detection", alpha=0.7, align='center')
    plt.title("Average Total Time Comparison")
    plt.ylabel("Average Total Time (seconds)")
    plt.xticks([0.8], ["Comparison"])
    plt.legend()
    plt.tight_layout()
    plt.savefig("figure/average_total_time_comparison.png")
    plt.show()

    # Print aborts for debug
    print("Total Aborts (Without Detection):", total_aborts_without_detection)
    print("Total Aborts (With Detection):", total_aborts_with_detection)

    # Average recoveries/aborts bar plots
    plot_single_bar(
        [average_recoveries_with_detection, average_recoveries_without_detection],
        ["With Detection", "Without Detection"],
        "Average Recoveries Comparison",
        "Average Recoveries",
        "figure/average_recoveries_comparison.png"
    )
    plot_single_bar(
        [average_aborts_with_detection, average_aborts_without_detection],
        ["With Detection", "Without Detection"],
        "Average Aborts Comparison",
        "Average Aborts",
        "figure/average_aborts_comparison.png"
    )

    # Box plots
    plot_boxplot(
        [data_with_detection["recoveries"], data_without_detection["recoveries"]],
        ["With Detection", "Without Detection"],
        "Recoveries Box Plot Comparison",
        "Recoveries",
        "lightblue",
        "figure/recoveries_boxplot_comparison.png"
    )
    plot_boxplot(
        [data_with_detection["aborts"], data_without_detection["aborts"]],
        ["With Detection", "Without Detection"],
        "Aborts Box Plot Comparison",
        "Aborts",
        "lightgreen",
        "figure/aborts_boxplot_comparison.png"
    )
    plot_boxplot(
        [data_with_detection["time_seconds"], data_without_detection["time_seconds"]],
        ["With Detection", "Without Detection"],
        "Time per POI Box Plot Comparison",
        "Time (seconds)",
        "lightcoral",
        "figure/time_per_poi_boxplot_comparison.png"
    )

    # Violin plots
    plot_violinplot(
        [data_with_detection["recoveries"], data_without_detection["recoveries"]],
        ["With Detection", "Without Detection"],
        "Recoveries Violin Plot Comparison",
        "Recoveries",
        "figure/recoveries_violinplot_comparison.png"
    )
    plot_violinplot(
        [data_with_detection["aborts"], data_without_detection["aborts"]],
        ["With Detection", "Without Detection"],
        "Aborts Violin Plot Comparison",
        "Aborts",
        "figure/aborts_violinplot_comparison.png"
    )
    plot_violinplot(
        [data_with_detection["time_seconds"], data_without_detection["time_seconds"]],
        ["With Detection", "Without Detection"],
        "Time per POI Violin Plot Comparison",
        "Time (seconds)",
        "figure/time_per_poi_violinplot_comparison.png"
    )

    # Violin plot del tempo totale
    plot_violinplot(
        [total_with_detection["time_seconds"], total_without_detection["time_seconds"]],
        ["With Detection", "Without Detection"],
        "Total Time Violin Plot Comparison",
        "Total Time (seconds)",
        "figure/total_time_violinplot_comparison.png"
    )

# ------------------------------------------------------------
# Entry Point
# ------------------------------------------------------------
if __name__ == "__main__":
    main()