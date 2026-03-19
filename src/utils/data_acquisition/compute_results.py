# ------------------------------------------------------------
# Final Statistical Report for Tour Data
# ------------------------------------------------------------

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

from scipy.stats import mannwhitneyu, bootstrap, spearmanr

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
FILE_WITH_DETECTION = "tour_times_with_detection.csv"
FILE_WITHOUT_DETECTION = "tour_times_without_detection.csv"
OUTPUT_DIR = "report_output"

sns.set_theme(style="whitegrid", context="talk")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ------------------------------------------------------------
# Utilities
# ------------------------------------------------------------
def mmss_to_seconds(value):
    """Convert 'MM:SS' to seconds."""
    if pd.isna(value):
        return np.nan

    value = str(value).strip()
    if ":" not in value:
        return np.nan

    parts = value.split(":")
    if len(parts) != 2:
        return np.nan

    try:
        minutes = int(parts[0])
        seconds = int(parts[1])
        return minutes * 60 + seconds
    except ValueError:
        return np.nan


def cliffs_delta(x, y):
    """
    Cliff's Delta effect size.
    """
    x = np.asarray(x)
    y = np.asarray(y)

    if len(x) == 0 or len(y) == 0:
        return np.nan

    greater = 0
    lower = 0

    for xi in x:
        greater += np.sum(xi > y)
        lower += np.sum(xi < y)

    return (greater - lower) / (len(x) * len(y))


def interpret_cliffs_delta(d):
    if pd.isna(d):
        return "n/a"

    ad = abs(d)
    if ad < 0.147:
        return "trascurabile"
    elif ad < 0.33:
        return "piccolo"
    elif ad < 0.474:
        return "medio"
    else:
        return "grande"


def rank_biserial_from_u(u_stat, n1, n2):
    if n1 == 0 or n2 == 0:
        return np.nan
    return 1 - (2 * u_stat) / (n1 * n2)


def bootstrap_ci_mean_diff(x, y, confidence_level=0.95, n_resamples=5000, random_state=42):
    """
    Bootstrap CI for mean(x) - mean(y).
    Returns NaN if one sample has fewer than 2 observations.
    """
    x = np.asarray(pd.Series(x).dropna())
    y = np.asarray(pd.Series(y).dropna())

    if len(x) < 2 or len(y) < 2:
        return np.nan, np.nan

    def statistic(a, b, axis=-1):
        return np.mean(a, axis=axis) - np.mean(b, axis=axis)

    res = bootstrap(
        (x, y),
        statistic,
        paired=False,
        vectorized=False,
        confidence_level=confidence_level,
        n_resamples=n_resamples,
        random_state=random_state,
        method="percentile"
    )

    return res.confidence_interval.low, res.confidence_interval.high


def descriptive_stats(series):
    series = pd.Series(series).dropna()

    if len(series) == 0:
        return {
            "n": 0,
            "mean": np.nan,
            "std": np.nan,
            "median": np.nan,
            "q1": np.nan,
            "q3": np.nan,
            "iqr": np.nan,
            "min": np.nan,
            "max": np.nan,
        }

    q1 = series.quantile(0.25)
    q3 = series.quantile(0.75)

    return {
        "n": int(series.count()),
        "mean": series.mean(),
        "std": series.std(ddof=1) if len(series) > 1 else np.nan,
        "median": series.median(),
        "q1": q1,
        "q3": q3,
        "iqr": q3 - q1,
        "min": series.min(),
        "max": series.max(),
    }


def save_table(df, filename):
    path = os.path.join(OUTPUT_DIR, filename)
    df.to_csv(path, index=False)
    return path


def savefig(filename):
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, filename), dpi=200, bbox_inches="tight")
    plt.close()


# ------------------------------------------------------------
# Data Loading
# ------------------------------------------------------------
def load_data_with_tour_id(file_path, condition_label):
    """
    Load CSV and assign a tour_id based on TOTAL rows.
    Assumption: each tour ends with poi_name == 'TOTAL'.
    """
    df = pd.read_csv(file_path, skip_blank_lines=True)
    df = df.dropna(how="all").copy()

    # Normalize poi_name
    df["poi_name"] = df["poi_name"].astype(str).str.strip()

    # Numeric cleanup
    for col in ["recoveries", "aborts", "poi_number"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    if "recoveries" in df.columns:
        df["recoveries"] = df["recoveries"].fillna(0)
    else:
        df["recoveries"] = 0

    if "aborts" in df.columns:
        df["aborts"] = df["aborts"].fillna(0)
    else:
        df["aborts"] = 0

    # Time parsing
    if "time_mmss" in df.columns:
        df["time_seconds"] = df["time_mmss"].apply(mmss_to_seconds)
    else:
        df["time_seconds"] = np.nan

    # Remove decorative lines
    df = df[df["poi_name"] != "############## END ##############"].copy()

    df["condition"] = condition_label

    # Assign tour_id
    tour_ids = []
    current_tour = 1

    for _, row in df.iterrows():
        tour_ids.append(current_tour)
        if row["poi_name"] == "TOTAL":
            current_tour += 1

    df["tour_id"] = tour_ids

    return df


def split_poi_and_total(full_df):
    total_rows = full_df[full_df["poi_name"] == "TOTAL"].copy()

    poi_rows = full_df[
        (full_df["poi_name"] != "TOTAL") &
        (~full_df["time_seconds"].isna())
    ].copy()

    return poi_rows, total_rows


def build_tour_level_dataset(full_df):
    """
    Build a tour-level dataset:
    - total_time_seconds from TOTAL row
    - recoveries/aborts from sum of POI rows
    """
    poi_rows, total_rows = split_poi_and_total(full_df)

    poi_agg = (
        poi_rows.groupby(["condition", "tour_id"], as_index=False)
        .agg(
            recoveries=("recoveries", "sum"),
            aborts=("aborts", "sum"),
            poi_time_sum=("time_seconds", "sum"),
            n_poi=("poi_number", "count")
        )
    )

    total_agg = (
        total_rows.groupby(["condition", "tour_id"], as_index=False)
        .agg(total_time_seconds=("time_seconds", "first"))
    )

    tours = poi_agg.merge(
        total_agg,
        on=["condition", "tour_id"],
        how="left"
    )

    # fallback if TOTAL is missing
    tours["total_time_seconds"] = tours["total_time_seconds"].fillna(tours["poi_time_sum"])

    return tours, poi_rows, total_rows


# ------------------------------------------------------------
# Statistics
# ------------------------------------------------------------
def compare_groups(x, y, metric_name, alternative="two-sided"):
    x = pd.Series(x).dropna().values
    y = pd.Series(y).dropna().values

    result = {
        "metric": metric_name,
        "n_with": len(x),
        "n_without": len(y),
        "u_stat": np.nan,
        "p_value": np.nan,
        "rank_biserial": np.nan,
        "cliffs_delta": np.nan,
        "cliffs_interpretation": "n/a",
        "mean_diff_with_minus_without": np.nan,
        "ci95_low": np.nan,
        "ci95_high": np.nan,
        "alternative": alternative,
        "note": ""
    }

    if len(x) == 0 or len(y) == 0:
        result["note"] = "Uno dei due gruppi è vuoto."
        return result

    result["mean_diff_with_minus_without"] = np.mean(x) - np.mean(y)

    if len(x) >= 2 and len(y) >= 2:
        test = mannwhitneyu(x, y, alternative=alternative)
        result["u_stat"] = test.statistic
        result["p_value"] = test.pvalue
        result["rank_biserial"] = rank_biserial_from_u(test.statistic, len(x), len(y))
        result["cliffs_delta"] = cliffs_delta(x, y)
        result["cliffs_interpretation"] = interpret_cliffs_delta(result["cliffs_delta"])
        ci_low, ci_high = bootstrap_ci_mean_diff(x, y)
        result["ci95_low"] = ci_low
        result["ci95_high"] = ci_high
    else:
        result["note"] = "Campione insufficiente per test inferenziale o bootstrap."

    return result


def build_descriptive_table(tours_df):
    rows = []

    for condition in tours_df["condition"].unique():
        subset = tours_df[tours_df["condition"] == condition]

        for metric in ["total_time_seconds", "recoveries", "aborts"]:
            stats = descriptive_stats(subset[metric])
            row = {
                "condition": condition,
                "metric": metric,
                **stats
            }
            rows.append(row)

    return pd.DataFrame(rows)


# ------------------------------------------------------------
# Plotting
# ------------------------------------------------------------
def plot_box_strip(tours_df, metric, title, ylabel, filename):
    plt.figure(figsize=(9, 6))
    sns.boxplot(data=tours_df, x="condition", y=metric, width=0.5)
    sns.stripplot(data=tours_df, x="condition", y=metric, size=7, alpha=0.7)
    plt.title(title)
    plt.xlabel("")
    plt.ylabel(ylabel)
    savefig(filename)


def plot_ecdf(tours_df, metric, title, xlabel, filename):
    plt.figure(figsize=(9, 6))
    sns.ecdfplot(data=tours_df, x=metric, hue="condition", linewidth=2.5)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Cumulative probability")
    savefig(filename)


def plot_poi_means_with_ci(poi_df, metric, title, ylabel, filename):
    plt.figure(figsize=(11, 6))
    sns.pointplot(
        data=poi_df,
        x="poi_number",
        y=metric,
        hue="condition",
        errorbar=("ci", 95),
        dodge=0.25,
        markers="o",
        linestyles="-"
    )
    plt.title(title)
    plt.xlabel("POI")
    plt.ylabel(ylabel)
    savefig(filename)


def plot_delta_heatmap(poi_df, metric, title, filename):
    summary = (
        poi_df.groupby(["condition", "poi_number"])[metric]
        .mean()
        .reset_index()
        .pivot(index="poi_number", columns="condition", values=metric)
    )

    if {"With Detection", "Without Detection"}.issubset(summary.columns):
        summary["delta"] = summary["With Detection"] - summary["Without Detection"]
        heat = summary[["delta"]]
    else:
        heat = summary

    # Controllo se ci sono dati validi
    if heat.isnull().all().all() or heat.empty:
        print(f"[WARNING] Heatmap '{filename}' non salvata: dati insufficienti o mancanti.")
        return

    print("plotting the figure")
    plt.figure(figsize=(5, 9))
    sns.heatmap(heat, annot=True, fmt=".2f", cmap="coolwarm", center=0)
    plt.title(title)
    plt.xlabel("")
    plt.ylabel("POI")
    #plt.show()
    savefig(filename)


def plot_bar_with_ci_from_summary(poi_df, metric, title, ylabel, filename):
    plt.figure(figsize=(11, 6))
    sns.barplot(
        data=poi_df,
        x="poi_number",
        y=metric,
        hue="condition",
        errorbar=("ci", 95),
        capsize=0.1
    )
    plt.title(title)
    plt.xlabel("POI")
    plt.ylabel(ylabel)
    savefig(filename)


# ------------------------------------------------------------
# Reporting
# ------------------------------------------------------------
def print_dataset_checks(full_with, full_without, tours_df, total_all):
    print("\n" + "=" * 80)
    print("CHECK DATASET")
    print("=" * 80)

    print("\nNumero righe TOTAL per file:")
    print("With Detection:", (full_with["poi_name"] == "TOTAL").sum())
    print("Without Detection:", (full_without["poi_name"] == "TOTAL").sum())

    print("\nNumero di tour per condizione:")
    print(tours_df.groupby("condition")["tour_id"].nunique())

    print("\nNumero righe tour-level per condizione:")
    print(tours_df.groupby("condition").size())

    print("\nNumero righe TOTAL aggregate per condizione:")
    print(total_all.groupby("condition").size())


def print_interpretation(test_df):
    print("\n" + "=" * 80)
    print("INFERENTIAL RESULTS")
    print("=" * 80)

    for _, row in test_df.iterrows():
        print(f"\nMetrica: {row['metric']}")
        print(f"  n_with = {row['n_with']}, n_without = {row['n_without']}")
        print(f"  U = {row['u_stat']}")
        print(f"  p-value = {row['p_value']}")
        print(f"  Diff media (With - Without) = {row['mean_diff_with_minus_without']}")
        print(f"  CI95% diff media = [{row['ci95_low']}, {row['ci95_high']}]")
        print(f"  Cliff's delta = {row['cliffs_delta']} ({row['cliffs_interpretation']})")
        print(f"  Rank-biserial = {row['rank_biserial']}")

        if row["note"]:
            print(f"  Nota: {row['note']}")
        else:
            if pd.notna(row["p_value"]) and row["p_value"] < 0.05:
                direction = "inferiore" if row["mean_diff_with_minus_without"] < 0 else "superiore"
                print(f"  Interpretazione: differenza statisticamente significativa; With Detection è mediamente {direction}.")
            else:
                print("  Interpretazione: non emerge una differenza statisticamente significativa.")


def correlation_summary(tours_df):
    print("\n" + "=" * 80)
    print("CORRELAZIONI")
    print("=" * 80)

    for condition in tours_df["condition"].unique():
        subset = tours_df[tours_df["condition"] == condition]

        if len(subset) < 2:
            print(f"{condition}: campione insufficiente per Spearman.")
            continue

        rho, p = spearmanr(subset["recoveries"], subset["total_time_seconds"], nan_policy="omit")
        print(f"{condition}: Spearman rho(time, recoveries) = {rho:.3f}, p = {p:.6f}")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main():
    # Load full datasets
    full_with = load_data_with_tour_id(FILE_WITH_DETECTION, "With Detection")
    full_without = load_data_with_tour_id(FILE_WITHOUT_DETECTION, "Without Detection")

    full_all = pd.concat([full_with, full_without], ignore_index=True)

    # Build datasets
    tours_df, poi_all, total_all = build_tour_level_dataset(full_all)

    # Debug / sanity checks
    print_dataset_checks(full_with, full_without, tours_df, total_all)

    print("\nPrime righe tours_df:")
    print(tours_df.head())

    # --------------------------------------------------------
    # Descriptive statistics
    # --------------------------------------------------------
    descriptive_table = build_descriptive_table(tours_df)
    save_table(descriptive_table, "descriptive_statistics.csv")

    print("\nStatistiche descrittive:")
    print(descriptive_table.round(3))

    # --------------------------------------------------------
    # Inferential statistics
    # --------------------------------------------------------
    tests = []

    for metric in ["total_time_seconds", "recoveries", "aborts"]:
        x = tours_df.loc[tours_df["condition"] == "With Detection", metric]
        y = tours_df.loc[tours_df["condition"] == "Without Detection", metric]
        tests.append(compare_groups(x, y, metric, alternative="two-sided"))

    test_df = pd.DataFrame(tests)
    save_table(test_df, "inferential_tests.csv")
    print_interpretation(test_df)

    # --------------------------------------------------------
    # Correlations
    # --------------------------------------------------------
    correlation_summary(tours_df)

    # --------------------------------------------------------
    # POI summary table
    # --------------------------------------------------------
    poi_summary = (
        poi_all.groupby(["condition", "poi_number"], as_index=False)
        .agg(
            mean_time_seconds=("time_seconds", "mean"),
            median_time_seconds=("time_seconds", "median"),
            mean_recoveries=("recoveries", "mean"),
            mean_aborts=("aborts", "mean"),
            n=("time_seconds", "count")
        )
    )
    save_table(poi_summary, "poi_summary.csv")

    # --------------------------------------------------------
    # Plots - tour level
    # --------------------------------------------------------
    plot_box_strip(
        tours_df,
        metric="total_time_seconds",
        title="Total time per tour",
        ylabel="Total time (seconds)",
        filename="01_total_time_box_strip.png"
    )

    plot_ecdf(
        tours_df,
        metric="total_time_seconds",
        title="ECDF of total time",
        xlabel="Total time (seconds)",
        filename="02_total_time_ecdf.png",
    )

    plot_box_strip(
        tours_df,
        metric="recoveries",
        title="Recoveries per tour",
        ylabel="Recoveries",
        filename="03_recoveries_box_strip.png"
    )

    plot_box_strip(
        tours_df,
        metric="aborts",
        title="Aborts per tour",
        ylabel="Aborts",
        filename="04_aborts_box_strip.png"
    )

    # --------------------------------------------------------
    # Plots - POI level
    # --------------------------------------------------------
    plot_poi_means_with_ci(
        poi_all,
        metric="time_seconds",
        title="Average time per POI with 95% CI",
        ylabel="Average time (seconds)",
        filename="05_poi_time_pointplot_ci.png"
    )

    plot_poi_means_with_ci(
        poi_all,
        metric="recoveries",
        title="Average recoveries per POI with 95% CI",
        ylabel="Average recoveries",
        filename="06_poi_recoveries_pointplot_ci.png"
    )

    plot_poi_means_with_ci(
        poi_all,
        metric="aborts",
        title="Average aborts per POI with 95% CI",
        ylabel="Average aborts",
        filename="07_poi_aborts_pointplot_ci.png"
    )

    plot_bar_with_ci_from_summary(
        poi_all,
        metric="time_seconds",
        title="Barplot average time per POI with 95% CI",
        ylabel="Average time (seconds)",
        filename="08_poi_time_bar_ci.png"
    )

    plot_delta_heatmap(
        poi_all,
        metric="recoveries",
        title="Delta average recoveries per POI (With - Without)",
        filename="09_heatmap_delta_recoveries.png"
    )

    plot_delta_heatmap(
        poi_all,
        metric="aborts",
        title="Delta average aborts per POI (With - Without)",
        filename="10_heatmap_delta_aborts.png"
    )

    print("\n" + "=" * 80)
    print(f"All outputs have been saved to: {OUTPUT_DIR}")
    print("=" * 80)


# ------------------------------------------------------------
# Entry point
# ------------------------------------------------------------
if __name__ == "__main__":
    main()