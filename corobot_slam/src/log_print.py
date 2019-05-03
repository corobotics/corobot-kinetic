import numpy
import math
import matplotlib.pyplot as plt

from collections import namedtuple


def restore_record(log_file):
    restored_data = dict()

    for line in log_file:
        if line.__contains__("Start"):
            batch = int(line.split("Start")[0].strip().split("Batch")[1])
            restored_data[batch] = dict()
            # Store robot pose and cov matrix.
            robot_info = namedtuple("Robot", "pose cov")
            robot_info.pose = numpy.matrix(log_file.readline().split(":")[1].strip())
            robot_info.cov = numpy.matrix(log_file.readline().split(":")[1].strip())
            restored_data[batch]["robot"] = robot_info

            # Store landmarks' poses and cov matrices.
            landmark_info = dict()
            line = log_file.readline()
            while line.__contains__("Landmark") and line.__contains__("location"):
                entity_info = namedtuple("Landmark", "name loc cov")
                entity_info.name = line.split("location")[0].strip().split("Landmark")[1].strip()
                # entity_info.name = line.split("Landmark")[1].strip()
                entity_info.loc = numpy.matrix(line.split(":")[1].strip())
                entity_info.cov = numpy.matrix(log_file.readline().split(":")[1].strip())
                landmark_info[entity_info.name] = entity_info
                line = log_file.readline()
            restored_data[batch]["landmarks"] = landmark_info

    return restored_data


def print_scatter(data, title):
    r_x = list()
    r_y = list()
    r_size = list()

    # Print robot pose.
    for batch in data.keys():
        r_x.append(data[batch]["robot"].pose[0].item())
        r_y.append(data[batch]["robot"].pose[1].item())
        one_size = math.sqrt(data[batch]["robot"].cov[0, 0].item() ** 2
                             + data[batch]["robot"].cov[1, 1].item() ** 2) * 10
        r_size.append(one_size)

    plt.scatter(r_x, r_y)
    plt.scatter(r_x, r_y, r_size, alpha=0.3)
    plt.title(title)
    plt.savefig(title + ".png")
    plt.close()


def fetch_data(data):
    landmarks = dict()
    for batch in data.keys():
        landmarks_onebatch = data[batch]["landmarks"]
        for landmark_name in landmarks_onebatch.keys():
            variance = math.sqrt(((landmarks_onebatch[landmark_name].cov[0, 0].item() * 6) ** 3) ** 2
                                 + ((landmarks_onebatch[landmark_name].cov[1, 1].item() * 6) ** 3) ** 2)
            if landmark_name not in landmarks:
                landmarks[landmark_name] = [variance]
            else:
                landmarks[landmark_name].append(variance)

    return landmarks


def print_linechart(s_data, t_data, title, lm_idx=None):
    s_landmarks = fetch_data(s_data)
    t_landmarks = fetch_data(t_data)

    #
    if lm_idx is not None:
        if lm_idx in t_landmarks.keys() and lm_idx in s_landmarks.keys():
            batches = [i for i in range(len(t_landmarks[lm_idx]))]
            fig, ax1 = plt.subplots()
            color = "tab:red"
            ax1.set_xlabel("Batches")
            ax1.set_ylabel("Smart Variances")
            ax1.plot(batches[-1 * len(s_landmarks[lm_idx]):], s_landmarks[lm_idx], color=color)
            ax1.tick_params(axis='y', labelcolor=color)

            ax2 = ax1.twinx()
            color = "tab:blue"
            ax2.set_ylabel("Traditional Variances")
            ax2.plot(batches[-1 * len(t_landmarks[lm_idx]):], t_landmarks[lm_idx], color=color)
            ax2.tick_params(axis='y', labelcolor=color)

            plt.title(title + " Landmark " + lm_idx + " variances")
            fig.tight_layout()
            plt.savefig("lm_" + lm_idx + "_variance.png")
            plt.close()
    else:
        for lm_idx in t_landmarks.keys():
            if lm_idx in t_landmarks.keys() and lm_idx in s_landmarks.keys():
                batches = [i for i in range(len(t_landmarks[lm_idx]))]
                fig, ax1 = plt.subplots()
                color = "tab:red"
                ax1.set_xlabel("Batches")
                ax1.set_ylabel("Smart Variances")
                ax1.plot(batches[-1 * len(s_landmarks[lm_idx]):], s_landmarks[lm_idx], color=color)
                ax1.tick_params(axis='y', labelcolor=color)

                ax2 = ax1.twinx()
                color = "tab:blue"
                ax2.set_ylabel("Traditional Variances")
                ax2.plot(batches[-1 * len(t_landmarks[lm_idx]):], t_landmarks[lm_idx], color=color)
                ax2.tick_params(axis='y', labelcolor=color)

                plt.title(title + " Landmark " + lm_idx + " variances")
                fig.tight_layout()
                plt.savefig("lm_" + lm_idx + "_variance.png")
                plt.close()


def print_number_lms(s_log, t_log):
    batches = list()
    s_lms, t_lms = list(), list()
    for line in s_log:
        if line.__contains__("Start"):
            s_counter = 0
            batch = int(line.split("Start")[0].strip().split("Batch")[1])
            line = s_log.readline()
            line = s_log.readline()
            line = s_log.readline()
            while line.__contains__("Landmark") and line.__contains__("location"):
                s_counter += 1
                line = s_log.readline()
                line = s_log.readline()
            batches.append(batch)
            s_lms.append(s_counter)

    for line in t_log:
        if line.__contains__("Start"):
            t_counter = 0
            line = t_log.readline()
            line = t_log.readline()
            line = t_log.readline()
            while line.__contains__("Landmark") and line.__contains__("location"):
                t_counter += 1
                line = t_log.readline()
                line = t_log.readline()
            t_lms.append(t_counter)


    fig, ax = plt.subplots()
    color = "tab:red"
    ax.set_xlabel("Batches")
    ax.plot(batches, s_lms, color=color, label="Smart SLAM")

    color = "tab:blue"
    ax.plot(batches, t_lms, color=color, label="Traditional SLAM")

    plt.legend()
    plt.title("Number of integrated landmarks")
    fig.tight_layout()
    plt.savefig("lm_integrate.png")
    plt.close()


def print_figs():
    s_log, t_log = open("smart_slam_log_s1.txt"), open("traditional_slam_log_s1.txt")
    s_data = restore_record(s_log)
    print_scatter(s_data, "Smart SLAM robot location with variance")
    t_data = restore_record(t_log)
    print_scatter(t_data, "Traditional SLAM robot location with variance")
    print_linechart(s_data, t_data, "SLAM")
    print_number_lms(s_log, t_log)


if __name__ == "__main__":
    print_figs()
