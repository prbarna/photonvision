import { expect } from "@playwright/test";
import { test } from "./fixtures.ts";

test("Camera Gain Slider won't go past max or min", async ({ page }) => {
  await page.goto("http://localhost:5800/#/dashboard");
  await page.locator("div").filter({ hasText: "Set up some cameras to get started!" }).nth(2).press("Escape");

  const gainRow = page.locator("div.d-flex").filter({ has: page.getByText("Camera Gain", { exact: true }) });
  const gainInput = gainRow.locator('input[type="number"]');
  const increment = gainRow.getByRole("button", { name: /appended action/i });
  const decrement = gainRow.getByRole("button", { name: /prepended action/i });

  // Fill in Camera Gain text field with 1000
  await gainInput.fill("1000");
  await gainInput.press("Enter");
  await expect(gainInput).toHaveValue("100");

  // Try using buttons to go past the max
  await increment.click();
  await expect(gainInput).toHaveValue("100");

  // Make sure the value is actually properly limited, not just visually
  await decrement.click();
  await expect(gainInput).toHaveValue("99");

  await gainInput.fill("-10");
  await gainInput.press("Enter");
  await expect(gainInput).toHaveValue("0");

  await decrement.click();
  await expect(gainInput).toHaveValue("0");

  // Make sure the value is actually properly limited, not just visually
  await increment.click();
  await expect(gainInput).toHaveValue("1");

  // Make sure that the guard actually prevents value setting, instead of just reverting the value
  // This can be ensured by making sure the Camera Gain field doesn't disappear (disappears when the value is -1)
  await decrement.click();
  await decrement.click();
  await expect(gainInput).toHaveValue("0");

  await expect(page.getByText("Camera Gain", { exact: true })).toBeVisible();
});
