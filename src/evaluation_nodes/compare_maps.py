#!/usr/bin/env python3
from __future__ import annotations

from csv import reader as csv_reader
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance_matrix


class FieldMap:
    def __init__(self, objects: dict[str, np.ndarray]):
        self.objects = objects

    def compute_score_with(
        self, other: FieldMap, object_key: str
    ) -> tuple[float, tuple[np.ndarray, np.ndarray]]:
        """
        Calculate the score according to the rules at:
        https://www.fieldrobot.com/event/index.php/contest-hybrid/task-h2/
        """
        score = 0.0

        objects_a = self.objects[object_key]
        objects_b = other.objects[object_key]

        distances = distance_matrix(objects_a, objects_b)
        matches = linear_sum_assignment(distances)

        for a_match, b_match in zip(*matches):
            distance = distances[a_match, b_match] * 100

            if distance <= 2.0:
                score += 15
            elif 2.0 < distance <= 37.5:
                score += 15.56 - 0.2817 * distance
            else:
                # Regarded as FP
                score -= 5

        # Negative score for additional FP
        if len(objects_a) > len(objects_b):
            score -= (len(objects_a) - len(objects_b)) * 5

        return score, matches

    def plot_matches_with(
        self,
        other: FieldMap,
        this_name: str,
        other_name: str,
        weed_matches: tuple[np.ndarray, np.ndarray],
        litter_matches: tuple[np.ndarray, np.ndarray],
        weed_score: float,
        litter_score: float,
        figsize: tuple[int, int] = (10, 10),
    ) -> plt.figure:
        plt.plot()
        plt.figure(figsize=figsize)
        plt.gca().axis("equal")

        _labels = []
        _lines = []

        def _add_objects(objects: np.ndarray, legend: str, **kwargs) -> None:
            _lines.append(plt.scatter(objects[:, 0], objects[:, 1], **kwargs))
            _labels.append(legend)

        # Add crop visualisation
        if self.objects["crop"].shape[0] > 0:
            crop = self.objects["crop"]
        else:
            crop = other.objects["crop"]

        _add_objects(
            crop,
            f"crop plants",
            color="g",
            marker=".",
        )

        _add_objects(
            self.objects["weed"],
            f"weeds_{this_name}",
            color="r",
            marker=".",
            s=100,
            alpha=0.5,
        )
        _add_objects(
            other.objects["weed"],
            f"weeds_{other_name}",
            color="r",
            marker="*",
            s=100,
            alpha=0.5,
        )
        _add_objects(
            self.objects["litter"],
            f"litter_{this_name}",
            color="b",
            marker=".",
            s=100,
            alpha=0.5,
        )
        _add_objects(
            other.objects["litter"],
            f"litter_{other_name}",
            color="b",
            marker="*",
            s=100,
            alpha=0.5,
        )

        # Location markers
        if (
            self.objects["location_marker_a"].shape[0] > 0
            and self.objects["location_marker_b"].shape[0] > 0
        ):
            location_marker_a = self.objects["location_marker_a"]
            location_marker_b = self.objects["location_marker_b"]
        else:
            location_marker_a = other.objects["location_marker_a"]
            location_marker_b = other.objects["location_marker_b"]

        plt.scatter(
            location_marker_a[:, 0],
            location_marker_a[:, 1],
            color="r",
            marker=".",
            alpha=0,
        )  # just to extend the axis of the plot
        plt.text(
            location_marker_a[:, 0],
            location_marker_a[:, 1],
            "A",
            bbox={"facecolor": "red", "alpha": 0.5, "pad": 10},
            ha="center",
            va="center",
        )

        plt.scatter(
            location_marker_b[:, 0],
            location_marker_b[:, 1],
            color="r",
            marker=".",
            alpha=0,
        )  # just to extend the axis of the plot
        plt.text(
            location_marker_b[:, 0],
            location_marker_b[:, 1],
            "B",
            bbox={"facecolor": "red", "alpha": 0.5, "pad": 10},
            ha="center",
            va="center",
        )

        def _draw_matches(
            matches: tuple[np.ndarray, np.ndarray], object_key: str
        ) -> None:
            for i, j in zip(*matches):
                distance = np.linalg.norm(
                    self.objects[object_key][i] - other.objects[object_key][j]
                )

                if distance > 0.375:
                    continue

                plt.plot(
                    [self.objects[object_key][i][0], other.objects[object_key][j][0]],
                    [self.objects[object_key][i][1], other.objects[object_key][j][1]],
                    linewidth=1,
                    markersize=12,
                    color="k",
                )
                plt.text(
                    *(self.objects[object_key][i] + 0.05),
                    f"{distance*100:.0f} cm",
                    size=6,
                )

        _draw_matches(weed_matches, "weed")
        _draw_matches(litter_matches, "litter")

        plt.legend(_lines, _labels)

        # Draw score
        txt = (
            f"Weed score: {weed_score:.2f} \n"
            f"Litter score: {litter_score:.2f} \n"
            f"Total score: {max(0, weed_score+litter_score):.2f}"
        )

        plt.figtext(
            0.5,
            0.01,
            txt,
            ha="center",
            fontsize=10,
            bbox={"facecolor": "orange", "alpha": 0.5, "pad": 5},
        )

        plt.grid()
        return plt

    @classmethod
    def from_csv(cls, csv_path: Path) -> FieldMap:
        with csv_path.open("r") as fs:
            reader = csv_reader(fs)

            objects = {
                "location_marker_a": np.array([]).reshape(0, 2),
                "location_marker_b": np.array([]).reshape(0, 2),
                "crop": np.array([]).reshape(0, 2),
                "litter": np.array([]).reshape(0, 2),
                "weed": np.array([]).reshape(0, 2),
            }

            try:
                for i, line in enumerate(reader):
                    # Skip header
                    if i == 0 and line[0].lower() == "x":
                        continue

                    if line[2] in list(objects.keys()):
                        xy = np.array([[float(line[0]), float(line[1])]])
                        objects[line[2]] = np.concatenate((objects[line[2]], xy))
            except:
                print(f"Something is wrong with {csv_path}")
                print("Probably there is an empty line in your file")

        return FieldMap(objects)


if __name__ == "__main__":
    from argparse import ArgumentParser

    from rospkg import RosPack

    vmf_path = Path(RosPack().get_path("virtual_maize_field"))

    parser = ArgumentParser(
        description="Compare gt_detection map with a prediiction_detection map"
    )
    parser.add_argument(
        "--gt_path",
        type=Path,
        default=vmf_path / "gt/map.csv",
        help="Path of the ground truth csv file generated by the map generator",
    )
    parser.add_argument(
        "--pred_path",
        type=Path,
        default=vmf_path / "map/pred_map.csv",
        help="Path of the prediction csv file, generated by the competitor performing the mapping task",
    )
    parser.add_argument(
        "--out_map",
        type=Path,
        default=vmf_path / "map/evaluation_map.png",
        help="Output location for a visual map",
    )
    parser.add_argument(
        "--out_dpi", type=int, default=300, help="DPI of the output map"
    )
    parser.add_argument(
        "--show_map", action="store_true", help="Show evaluation map after generation."
    )
    args = parser.parse_args()

    gt = FieldMap.from_csv(args.gt_path)
    pred = FieldMap.from_csv(args.pred_path)

    weed_score, weed_matches = pred.compute_score_with(gt, "weed")
    litter_score, litter_matches = pred.compute_score_with(gt, "litter")

    fig = pred.plot_matches_with(
        gt, "pred", "gt", weed_matches, litter_matches, weed_score, litter_score
    )
    fig.savefig(args.out_map, dpi=args.out_dpi)

    print(f"Weed score: {weed_score:.2f}")
    print(f"Litter score: {litter_score:.2f}")
    print(f"Total score: {max(0, weed_score + litter_score):.2f}")

    if args.show_map:
        plt.show()

    exit(0)
