# SPDX-License-Identifier: GPL-2.0
dtbo-y += exynos/exynos9820-universal9820_s5100_rev1.dtbo
dtbo-y += exynos/exynos9820-smdk9820.dtbo
dtbo-y += exynos/exynos9820-universal9820_5g.dtbo
dtbo-y += exynos/exynos9825-universal9825_revC04.dtbo
dtbo-y += exynos/exynos9820-universal9820_revC04.dtbo
dtbo-y += exynos/exynos9825-universal9825_revC03.dtbo
dtbo-y += exynos/exynos9825-universal9825_revC06.dtbo
dtbo-y += exynos/exynos9820-universal9820_revC03.dtbo
dtbo-y += exynos/exynos9825-smdk9825.dtbo
dtbo-y += exynos/exynos9825-universal9825_5g.dtbo
dtbo-y += exynos/exynos9825-universal9825_rev5.dtbo
dtb-y += exynos/exynos5433-tm2.dtb
dtbo-y += exynos/exynos9825-universal9825_s5100_rev1.dtbo
dtbo-y += exynos/exynos9820-universal9820_revC06.dtbo
dtbo-y += exynos/exynos9825-universal9825_revC01.dtbo
dtbo-y += exynos/exynos9820-universal9820_rev5.dtbo
dtbo-y += exynos/exynos9820-universal9820_revC01.dtbo
dtb-y += exynos/exynos9825.dtb
dtb-y += exynos/exynos7-espresso.dtb
dtb-y += exynos/exynos5433-tm2e.dtb
dtb-y += exynos/exynos9820.dtb
dtbo-y += samsung/exynos9825-m62_eur_open_01.dtbo
dtbo-y += samsung/exynos9825-m62_eur_open_02.dtbo
dtbo-y += samsung/exynos9825-m62_eur_open_00.dtbo
dtbo-y += samsung/exynos9825-m62_eur_open_03.dtbo

targets += dtbs
DTB_LIST  := $(dtb-y) $(dtbo-y)
always    := $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb*
