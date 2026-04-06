# Bond Valuation (CFA Level I Study Notes)

These notes are a standalone study guide focused on the bond valuation ideas most useful for CFA Level I: cash flow structure, pricing, yields, accrued interest, clean versus dirty price, and the price-yield relationship.

## Learning Goals

By the end of these notes, you should be able to:

- describe the basic cash flow pattern of a plain-vanilla bond
- price a coupon bond and a zero-coupon bond
- distinguish among coupon rate, current yield, and yield to maturity
- explain accrued interest and clean versus dirty price
- identify whether a bond trades at a premium, discount, or par
- explain why bond prices move inversely with yields
- apply quick exam rules without losing the underlying intuition

---

## 1. What Is a Bond?

A bond is a loan that investors make to an issuer such as a government, corporation, municipality, or agency. In return, the issuer promises two things:

1. periodic interest payments, called coupon payments
2. repayment of principal, usually called face value or par value, at maturity

At issuance, the bond contract specifies:

- face value
- coupon rate
- payment frequency
- maturity date
- any special features such as callability, convertibility, or putability

For basic CFA Level I valuation, we usually begin with an option-free fixed-rate bond.

### Key Terms

| Term | Meaning |
|------|---------|
| Face value / par value | Amount repaid at maturity |
| Coupon rate | Annual stated interest rate on face value |
| Coupon payment | Dollar coupon paid each period |
| Maturity | Date when principal is repaid |
| Issuer | Borrower |
| Bondholder | Investor/lender |
| Yield to maturity (YTM) | Discount rate that equates the bond's present value to its market price |

### Why Bonds Matter

Bonds matter because they are central to financing, valuation, portfolio construction, and macroeconomic analysis. Fixed income markets also help reveal market expectations about:

- future interest rates
- inflation
- credit risk
- economic growth

For CFA purposes, bond analysis usually starts with three questions:

1. What are the bond's promised cash flows?
2. What discount rate should be used?
3. How does the price respond when required return changes?

### Conceptual Example

Suppose you lend a company \$1,000 for five years and it agrees to pay you 6% annual interest with semiannual payments.

- You will receive \$30 every six months.
- At the end of five years, you receive the final \$30 coupon plus the \$1,000 principal.

That is the basic structure of a coupon bond.

---

## 2. Bond Cash Flow Structure

A plain-vanilla coupon bond generates two types of cash flows:

1. coupon payments during the bond's life
2. principal repayment at maturity

If:

- $F$ = face value
- $c$ = annual coupon rate
- $m$ = coupon payments per year
- $T$ = years to maturity

then the coupon payment per period is:

$$
C = \frac{cF}{m}
$$

and the total number of coupon periods is:

$$
N = mT
$$

### Example: 5-Year, 6% Semiannual Bond

Let:

- $F = 1{,}000$
- $c = 6\%$
- $m = 2$
- $T = 5$

Then:

$$
C = \frac{0.06 \times 1{,}000}{2} = 30
$$

and:

$$
N = 2 \times 5 = 10
$$

So the investor receives:

- \$30 each half-year for 9 periods
- \$1,030 in the final period

The final payment is larger because it includes:

$$
\text{Final cash flow} = C + F
$$

### Why Payment Frequency Matters

The annual coupon rate does not tell you the actual dollar payment unless you know the payment frequency.

For the same 6% coupon bond on \$1,000 face value:

- annual payments: \$60 once per year
- semiannual payments: \$30 twice per year
- quarterly payments: \$15 four times per year

The total coupon over a year is still \$60, but timing changes valuation because earlier cash flows are worth more than later ones.

### Conceptual Example

Two bonds both have:

- face value = \$1,000
- coupon rate = 8%
- maturity = 4 years

Bond A pays annually. Bond B pays semiannually.

Bond A pays \$80 each year. Bond B pays \$40 every six months. Bond B returns more cash earlier, so if market discounting is consistent with the payment frequency, its valuation mechanics must reflect that timing.

### CFA Level I Reminder

For standard U.S. corporate and Treasury coupon bonds, semiannual compounding is common in exam-style pricing questions. Always match:

- coupon frequency
- discounting frequency
- number of periods

---

## 3. Coupon Bond Pricing

### Core Principle

The value of a bond is the present value of its future cash flows.

If the market requires yield $y$, then each coupon and the principal repayment must be discounted back to today.

### General Pricing Formula

For a coupon bond:

$$
P = \sum_{k=1}^{N} \frac{C}{(1+y/m)^k} + \frac{F}{(1+y/m)^N}
$$

where:

- $P$ = bond price
- $C$ = coupon payment per period
- $F$ = face value
- $y$ = annual yield to maturity
- $m$ = compounding frequency
- $N$ = total number of periods

This formula simply says:

- discount every coupon
- discount the principal repayment
- add the present values together

### Closed-Form Formula

Because the coupon stream is an annuity, the coupon bond formula can also be written as:

$$
P = C \cdot \frac{1 - (1+y/m)^{-N}}{y/m} + F(1+y/m)^{-N}
$$

This version is faster for hand calculations and exam work.

### Worked Example

Price a 5-year, 6% semiannual bond when YTM is 7%.

Inputs:

- $F = 1{,}000$
- $c = 6\%$
- $y = 7\%$
- $m = 2$
- $T = 5$

First compute the coupon and number of periods:

$$
C = \frac{0.06 \times 1{,}000}{2} = 30
$$

$$
N = 2 \times 5 = 10
$$

Periodic yield:

$$
r = \frac{y}{m} = \frac{0.07}{2} = 0.035
$$

Now apply the formula:

$$
P = 30 \cdot \frac{1 - (1.035)^{-10}}{0.035} + 1{,}000(1.035)^{-10}
$$

$$
P \approx 958.42
$$

### Interpretation

The bond sells below par because:

- coupon rate = 6%
- required yield = 7%

The bond's coupon payments are too low relative to current market rates, so investors will only buy it at a discount.

### Fast Exam Rules

- If coupon rate $>$ YTM, then price $>$ par.
- If coupon rate $=$ YTM, then price $=$ par.
- If coupon rate $<$ YTM, then price $<$ par.

### Conceptual Example

Imagine the market suddenly requires 9% return on similar bonds, but your bond still pays a 6% coupon. The issuer cannot change the coupon after issuance, so the only way for the bond to offer a competitive return is for its price to fall.

That price drop creates a capital gain for a new investor who buys below par and receives par at maturity.

### Important Assumption

The bond valuation formula prices promised cash flows, not guaranteed cash flows. If the issuer has default risk, the market-required yield will include compensation for that risk.

---

## 4. Zero-Coupon Bonds

A zero-coupon bond makes no periodic coupon payments. The investor pays a discounted price today and receives face value at maturity.

So for a zero-coupon bond:

$$
C = 0
$$

and the pricing formula simplifies to:

$$
P_{\text{zero}} = \frac{F}{(1+y/m)^N}
$$

### Example

Price a 10-year zero-coupon bond with:

- $F = 1{,}000$
- $y = 5\%$
- $m = 2$

Then:

$$
N = 10 \times 2 = 20
$$

$$
P = \frac{1{,}000}{(1.025)^{20}} \approx 610.27
$$

### Why Zero-Coupon Bonds Matter

Zeros are extremely important conceptually:

- they eliminate reinvestment risk because there are no interim coupon payments
- they are the building blocks of fixed-income valuation
- any coupon bond can be viewed as a package of zero-coupon cash flows

That last idea is foundational. A coupon bond is just a collection of future payments, and each payment can be priced as if it were a small zero-coupon bond.

### Conceptual Example

If you know you need exactly \$50,000 in 8 years for tuition, a zero-coupon bond can be a cleaner match than a coupon bond. With a coupon bond, you receive cash before the liability date and must reinvest it. With a zero, you receive one lump sum at the end.

### CFA Level I Takeaway

For a given maturity, a zero-coupon bond has greater interest rate sensitivity than a coupon bond because all cash flow occurs at the final date.

---

## 5. Accrued Interest and Day Count Conventions

### Why Accrued Interest Exists

Bonds are often traded between coupon payment dates. If a seller has held the bond for part of the coupon period, that seller has earned part of the next coupon even though the payment has not yet been made.

The buyer compensates the seller for this earned portion through accrued interest.

### Formula

$$
\text{Accrued Interest} = C \times \frac{\text{Days since last coupon}}{\text{Days in coupon period}}
$$

### Example

Suppose:

- face value = \$1,000
- coupon rate = 6%
- semiannual coupon

Then the coupon per period is:

$$
C = \frac{0.06 \times 1{,}000}{2} = 30
$$

If 90 days have passed in a 180-day coupon period:

$$
\text{Accrued Interest} = 30 \times \frac{90}{180} = 15
$$

The buyer owes the seller \$15 of accrued interest.

### Economic Intuition

Without accrued interest, the seller would lose value by selling before the coupon date, and the buyer would receive a windfall. Accrued interest makes settlement fair between the two parties.

### Day Count Conventions

The day count convention determines how the fraction of the coupon period is measured.

| Convention | Description | Common Use |
|-----------|-------------|------------|
| 30/360 | Each month assumed to have 30 days, year assumed to have 360 days | Many corporate and municipal bonds |
| Actual/Actual | Uses actual calendar days in period and year | U.S. Treasuries |
| Actual/360 | Actual days elapsed, 360-day year | Money market instruments |
| Actual/365 | Actual days elapsed, 365-day year | Some international markets |

### Conceptual Example

A Treasury bond and a corporate bond can have the same coupon and maturity but slightly different accrued interest on the same settlement date because they use different day count conventions.

This is a favorite exam trap: the valuation logic may be identical, but the settlement amount differs.

### CFA Level I Reminder

Know the common convention associations:

- U.S. Treasuries: Actual/Actual
- many corporates: 30/360

---

## 6. Clean Price Versus Dirty Price

Because accrued interest is added at settlement, bonds effectively have two prices.

### Definitions

- Clean price: quoted price, excluding accrued interest
- Dirty price: full price paid by the buyer, including accrued interest

The relationship is:

$$
P_{\text{dirty}} = P_{\text{clean}} + \text{Accrued Interest}
$$

### Why Markets Quote Clean Price

If markets quoted dirty price, bond prices would mechanically rise between coupon dates as accrued interest builds up, then drop after the coupon is paid. That would obscure actual market-driven price changes.

Quoting clean price removes that mechanical sawtooth pattern and makes price comparisons more meaningful.

### Example

Suppose:

- clean price = \$980
- accrued interest = \$15

Then:

$$
P_{\text{dirty}} = 980 + 15 = 995
$$

The buyer actually pays \$995.

### Conceptual Example

A bond might appear "cheap" at a quoted clean price of 99.20, but if settlement occurs just before the coupon date, accrued interest could be large. The invoice amount can therefore be materially above the quoted price.

### Exam Tip

If the question asks:

- "quoted price" or "flat price" -> use clean price
- "full price," "invoice price," or "amount paid" -> use dirty price

---

## 7. Yield to Maturity

### Definition

Yield to maturity is the single discount rate that equates the present value of a bond's promised cash flows to its market price.

It solves:

$$
P_{\text{market}} = \sum_{k=1}^{N} \frac{C}{(1+y/m)^k} + \frac{F}{(1+y/m)^N}
$$

YTM is essentially the bond's internal rate of return if the bond is:

- purchased at the current market price
- held to maturity
- paid as promised
- and coupons are reinvested at the YTM

### Important Interpretation

YTM is not just "coupon income." It is a total return concept that incorporates:

- coupon income
- any capital gain or capital loss from the difference between price and par
- the time value of money

### Example

A 5-year, 6% semiannual bond trades at \$957.35. Since the price is below par, the bond is a discount bond, so its YTM must be greater than 6%.

Solving the pricing equation gives a YTM of about 7%.

### Why Solving for YTM Can Be Hard

When price is known and yield is unknown, the pricing equation cannot usually be rearranged into a simple closed-form solution for $y$. In practice, we solve it numerically or use a calculator.

### YTM Assumptions and Limitations

YTM is useful, but it is not perfect. It assumes:

1. the bond is held until maturity
2. all payments are made in full and on time
3. interim coupons are reinvested at the same YTM

That reinvestment assumption is especially important. If actual reinvestment rates differ, realized return will differ from YTM.

### Conceptual Example

Suppose a bond has a YTM of 8%, but after you buy it, market rates fall and you can reinvest coupons only at 4%. Your realized return will likely be below the original YTM if you hold to maturity.

### CFA Level I Takeaway

Use YTM as a standardized measure for comparing fixed-rate bonds, but remember that it is a model-implied return, not a guaranteed realized return.

---

## 8. Current Yield Versus YTM

Current yield is a simpler, incomplete yield measure:

$$
\text{Current Yield} = \frac{\text{Annual Coupon}}{P_{\text{clean}}}
$$

Since annual coupon equals $cF$, we can write:

$$
\text{Current Yield} = \frac{cF}{P_{\text{clean}}}
$$

### What Current Yield Captures

Current yield captures only the income component relative to the bond's price. It ignores:

- capital gain or loss as the bond moves toward par
- timing of cash flows
- reinvestment effects

### What YTM Captures

YTM captures:

- coupon income
- capital gain or capital loss
- time value of money
- reinvestment assumption

### Ordering Relationships

| Bond Type | Ordering |
|-----------|----------|
| Discount bond | Coupon rate < Current yield < YTM |
| Par bond | Coupon rate = Current yield = YTM |
| Premium bond | Coupon rate > Current yield > YTM |

### Why the Ordering Works

For a discount bond:

- price is below par
- annual coupon divided by price is therefore above the coupon rate
- but YTM is even higher because it also includes a capital gain as price moves toward par

For a premium bond, the logic reverses because the bond experiences a capital loss as it approaches par.

### Example

Suppose a 6% coupon bond trades at \$950.

Annual coupon:

$$
0.06 \times 1{,}000 = 60
$$

Current yield:

$$
\frac{60}{950} = 6.32\%
$$

Because the bond is below par, YTM must be above 6.32%.

### Conceptual Example

Two investors can both say "this bond yields 6%," but one may mean current yield while the other means YTM. These are not interchangeable statements. On the exam, always check which definition is being used.

### Exam Tip

If the question is about "total expected annualized return if held to maturity," YTM is usually the intended measure, not current yield.

---

## 9. Price-Yield Relationship

The bond price-yield relationship is one of the most important ideas in fixed income.

### 1. Inverse Relationship

Bond prices and yields move in opposite directions.

If required yield rises, discount rates rise, and the present value of future cash flows falls:

$$
y \uparrow \Rightarrow P \downarrow
$$

If required yield falls, present value rises:

$$
y \downarrow \Rightarrow P \uparrow
$$

### 2. Convexity

For an option-free bond, the price-yield curve is convex, not linear.

This means:

- a drop in yield increases price by more than an equal-sized rise in yield decreases price
- price sensitivity changes at different yield levels

This convexity is beneficial to investors holding option-free bonds.

### 3. Maturity Effect

Longer-maturity bonds are generally more sensitive to interest rate changes than shorter-maturity bonds, all else equal.

Why? More of their value depends on cash flows received far in the future, and those distant cash flows are more affected by discount rate changes.

### 4. Coupon Effect

Lower-coupon bonds are generally more sensitive to interest rate changes than higher-coupon bonds, all else equal.

Why? With lower coupons, a larger share of value comes from the final payment rather than earlier coupons, so cash flows are weighted further into the future.

### Extreme Case: Zero-Coupon Bonds

For a given maturity, a zero-coupon bond has the greatest sensitivity to yield changes because all cash is received at maturity.

### Conceptual Example

Compare two 10-year bonds:

- Bond X: 2% coupon
- Bond Y: 8% coupon

If yields rise by 1%, Bond X will generally suffer a larger price decline because its value depends more heavily on distant cash flows.

### CFA Level I Link

These ideas set up later topics such as:

- duration
- convexity
- interest rate risk

At Level I, the most important point is still the intuition: longer maturity and lower coupon usually mean greater interest rate risk.

---

## 10. Premium, Discount, and Par Bonds

Bond price relative to par depends on the relationship between coupon rate and YTM.

| Bond Type | Condition | Price Relationship |
|-----------|-----------|-------------------|
| Premium bond | Coupon rate > YTM | $P > F$ |
| Par bond | Coupon rate = YTM | $P = F$ |
| Discount bond | Coupon rate < YTM | $P < F$ |

### Why This Happens

If a bond's coupon is higher than what the market currently requires, the bond is attractive, so investors bid up its price above par.

If the coupon is lower than what the market requires, the bond is unattractive at par, so its price falls below par.

### Pull-to-Par Effect

As maturity approaches, a bond's price moves toward par value, assuming required yield remains unchanged.

- Premium bonds decline toward par.
- Discount bonds rise toward par.
- Par bonds remain near par.

This is often called the pull-to-par effect.

### Why Pull-to-Par Happens

At maturity, the principal repayment is fixed at face value. Therefore, as the maturity date gets closer, there is less uncertainty about that final value and less time for pricing differences to persist.

### Conceptual Example

Suppose a bond was issued years ago with an 8% coupon, but now similar bonds yield 5%.

- Investors like the high coupon.
- The bond trades above par.
- But if the bond is redeemed at only \$1,000, that premium cannot last forever.

Over time, the price must gradually decline toward par, offsetting some of the extra coupon income.

### Yield Measures Across Bond Types

For a premium bond:

$$
\text{Coupon Rate} > \text{Current Yield} > \text{YTM}
$$

For a discount bond:

$$
\text{Coupon Rate} < \text{Current Yield} < \text{YTM}
$$

For a par bond:

$$
\text{Coupon Rate} = \text{Current Yield} = \text{YTM}
$$

### Exam Tip

When given only price relative to par, you can often infer the relationship between coupon rate and YTM without doing any calculation.

---

## 11. Practical Exam Framework

When facing a CFA Level I bond pricing question, this sequence is usually the safest:

1. Identify face value, coupon rate, maturity, and coupon frequency.
2. Convert the annual coupon rate into coupon per period:

$$
C = \frac{cF}{m}
$$

3. Convert annual YTM into periodic yield:

$$
r = \frac{y}{m}
$$

4. Compute the number of periods:

$$
N = mT
$$

5. Price the bond using either:

$$
P = \sum_{k=1}^{N} \frac{C}{(1+r)^k} + \frac{F}{(1+r)^N}
$$

or:

$$
P = C \cdot \frac{1-(1+r)^{-N}}{r} + F(1+r)^{-N}
$$

6. If the bond is traded between coupon dates, add accrued interest to move from clean price to dirty price.

### Common Mistakes

- using annual coupon instead of coupon per period
- forgetting to divide YTM by payment frequency
- forgetting to multiply years by payment frequency to get total periods
- mixing clean price and dirty price
- confusing current yield with YTM
- ignoring day count convention in accrued interest problems

---

## 12. Conceptual Mini-Examples

### Example A: Why Does Price Fall When Yield Rises?

If investors now require 8% instead of 6%, existing 6% bonds are less attractive. Since the coupon is fixed by contract, the only adjustment mechanism is price. The price must fall until the lower purchase price compensates for the below-market coupon.

### Example B: Why Is a Zero More Volatile?

A 10-year zero-coupon bond pays nothing until year 10. All value is concentrated in one distant cash flow. If the discount rate changes, the value of that distant payment changes more sharply than the value of a bond that returns part of the cash earlier through coupons.

### Example C: Why Is Current Yield Incomplete?

A bond may have a current yield of 7%, but if it trades at a large premium and will lose value as it moves toward par, the investor's total return can be materially lower than 7%. That is why YTM is the more complete measure.

### Example D: Why Quote Clean Price Instead of Dirty Price?

Suppose nothing in the market changes between today and next week. The bond's dirty price can still rise simply because more accrued interest has accumulated. Quoting clean price strips out that mechanical effect and makes price movements easier to interpret.

---

## 13. Quick Recall Summary

- A bond is worth the present value of its future cash flows.
- Coupon per period:

$$
C = \frac{cF}{m}
$$

- Number of periods:

$$
N = mT
$$

- Coupon bond price:

$$
P = \sum_{k=1}^{N} \frac{C}{(1+y/m)^k} + \frac{F}{(1+y/m)^N}
$$

- Zero-coupon price:

$$
P_{\text{zero}} = \frac{F}{(1+y/m)^N}
$$

- Accrued interest:

$$
\text{AI} = C \times \frac{\text{Days since last coupon}}{\text{Days in coupon period}}
$$

- Dirty versus clean price:

$$
P_{\text{dirty}} = P_{\text{clean}} + \text{AI}
$$

- Current yield:

$$
\text{Current Yield} = \frac{cF}{P_{\text{clean}}}
$$

- Premium/par/discount rules:
  - coupon rate > YTM -> premium
  - coupon rate = YTM -> par
  - coupon rate < YTM -> discount

- Price-yield rule:
  - yields rise -> prices fall
  - yields fall -> prices rise

---

## 14. Final CFA Level I Takeaways

For Level I, the most testable bond valuation ideas are not just the formulas but the relationships behind them:

- price is the present value of promised cash flows
- yield is the market-required return
- coupon rate is fixed, but price adjusts
- accrued interest affects settlement value
- clean price is quoted, dirty price is paid
- current yield is incomplete, YTM is broader
- longer maturity and lower coupon increase interest rate sensitivity
- all option-free bonds exhibit an inverse and convex price-yield relationship

If you can explain those relationships in words and also execute the formulas correctly, you are in strong shape for most introductory fixed income valuation questions.

---

## References

1. CFA Institute, *CFA Program Curriculum Level I - Fixed Income*.
2. Fabozzi, F. J. *Bond Markets, Analysis, and Strategies*.
3. Tuckman, B. and Serrat, A. *Fixed Income Securities*.
