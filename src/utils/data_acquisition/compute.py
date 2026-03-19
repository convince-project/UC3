import csv
from datetime import timedelta

def parse_time_to_seconds(time_str):
    """Convert time in mm:ss format to seconds."""
    minutes, seconds = map(int, time_str.split(':'))
    return minutes * 60 + seconds

def calculate_averages(file_path):
    total_recoveries = 0
    total_aborts = 0
    total_time_seconds = 0
    tour_count = 0
    poi_times = {}

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if row[0] == "TOTAL":
                # Parse TOTAL time in mm:ss format
                total_time_seconds += parse_time_to_seconds(row[2])
                tour_count += 1
            elif row[0].startswith("#") or row[0] == "poi_name":
                # Skip headers and separators
                continue
            else:
                # Sum recoveries and aborts
                total_recoveries += int(row[3])
                total_aborts += int(row[4])

                # Calculate total time for each POI
                poi_name = row[0]
                time_seconds = parse_time_to_seconds(row[2])
                if poi_name not in poi_times:
                    poi_times[poi_name] = []
                poi_times[poi_name].append(time_seconds)

    # Calculate averages
    avg_recoveries = total_recoveries / tour_count
    avg_aborts = total_aborts / tour_count
    avg_total_time_seconds = total_time_seconds / tour_count

    # Convert average time back to mm:ss format
    avg_minutes = int(avg_total_time_seconds // 60)
    avg_seconds = int(avg_total_time_seconds % 60)
    avg_total_time = f"{avg_minutes:02}:{avg_seconds:02}"

    # Calculate average time per POI
    avg_poi_times = {}
    for poi_name, times in poi_times.items():
        avg_time_seconds = sum(times) / len(times)
        avg_minutes = int(avg_time_seconds // 60)
        avg_seconds = int(avg_time_seconds % 60)
        avg_poi_times[poi_name] = f"{avg_minutes:02}:{avg_seconds:02}"

    return avg_recoveries, avg_aborts, avg_total_time, avg_poi_times, tour_count

# Example usage
file_path = "tour_times_with_detection.csv"
avg_recoveries, avg_aborts, avg_total_time, avg_poi_times, tour_counts = calculate_averages(file_path)
print(f"Results without Detection (average of {tour_counts} tours):")
print(f"Average Recoveries: {avg_recoveries}")
print(f"Average Aborts: {avg_aborts}")
print(f"Average TOTAL Time: {avg_total_time}")
print("Average Time per POI:")
for poi, avg_time in avg_poi_times.items():
    print(f"  {poi}: {avg_time}")
