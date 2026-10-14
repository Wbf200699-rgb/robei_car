from app.anchor_calibration import solve_anchor_coordinates


def test_rectangular_four_anchor_solution() -> None:
    result = solve_anchor_coordinates({
        "AB": 224.5,
        "AC": 150.0,
        "AD": 167.0,
        "BC": 167.0,
        "BD": 150.0,
        "CD": 224.5,
    })
    assert result["quality"] == "full"
    coordinates = result["coordinates_cm"]
    assert abs(coordinates["C"]["x"] - 150.0) < 0.01
    assert abs(coordinates["B"]["x"] - 150.0) < 0.2
    assert abs(coordinates["D"]["x"]) < 0.2
    assert result["residual_cm"] < 0.5


def test_incomplete_solution_is_not_accepted() -> None:
    result = solve_anchor_coordinates({"AB": 100, "AC": 150})
    assert result["quality"] == "incomplete"
    assert "CD" in result["missing_pairs"]
